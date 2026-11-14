/* SPDX-License-Identifier: GPL-2.0-or-later */
/* Copyright GlassineOS contributors */
/* glassine-epdd-desktop — e-ink refresh daemon for X11 on the Note Air 1.
 *
 * Two backends, same daemon: the stock 4.4 kernel's /dev/ebc (Onyx driver), or
 * — when there is no /dev/ebc (the mainline kernel) — the userspace engine
 * epd-engine.c + epd-drm.c, which implements the same update semantics (per
 * pixel scheduling, collisions queue, buffer copied at submit, DU frame cut)
 * and streams the drive frames over DRM. See ebc_update.
 *
 * The panel only changes when /dev/ebc is told to (ioctl 0x700c). This daemon
 * watches the X root window with XDamage, copies changed pixels into the
 * /dev/ebc buffer (read from /dev/fb0, where Xorg's rotated shadow copy already
 * sits turned — see snap; XGetImage is the default) (portrait X
 * screen -> landscape panel, same mapping as glassine-epd-show's upright "ccw"), and
 * issues updates.
 *
 * GPU compositing (mainline): when X runs glamor on the GPU (modesetting
 * on vkms, see glassine-session), this daemon is the X compositing manager instead
 * (epd-comp.c): it redirects the top-level windows and composites them itself
 * on the Adreno, straight into panel-order grey (rotation, luma, the gamma LUT,
 * ARGB alpha in shaders). Damage then comes per window; pixels come from the
 * windows' own buffers, complete, not from a root the apps are drawing into.
 * Without a GPU (stock 4.4, the dummy X fallback) the root is read as before.
 *
 * The refresh decisions (which waveform, when to flash, cleanups, motion runs)
 * live in epd-policy.c: a pure state machine, host-tested (test/). This file is
 * its I/O: X damage, pen/touch evdev, the FIFO, and executing its actions.
 *
 * Damage is kept as a small list of rectangles (overlapping/nearby ones merged),
 * NOT one bounding box: a bounding box of e.g. a new window plus its panel task
 * button covers nearly the whole screen. Each burst (coalesced COALESCE_MS) is
 * sent per rectangle, waveform by size (fraction of the screen):
 * - small (<= du_max, pen strokes, typing, cursor): DU — fast, black/white only;
 * remembered and cleaned with GC16 after IDLE_GC_MS without damage
 * - medium: GC16 partial — correct greys, no flash (~0.5 s). While GC16s keep
 * coming (animation, scrolling) medium areas fall back to DU + cleanup
 * - large (>= flash_min, app switch, page turn): a flash of the area (inverted,
 * then real: see send) — leaves no ghosting
 * Each rectangle is first shrunk to the pixels that actually differ from what
 * the panel already shows (the /dev/ebc buffer is our shadow copy): apps
 * repaint far more than they change (Xournal++ re-renders its whole canvas after
 * every stroke), and unchanged repaints must not flash.
 * DU on a large area left pixels half-transitioned (grey targets / old content)
 * until the cleanup: the "repaint does not complete" artifacts.
 * - at start, or on "full" via the control FIFO /run/glassine-epdd-desktop.ctl:
 * full-screen flashing GC16
 * - ghosting: DU updates are counted per 32 px tile; at an idle cleanup the
 * tiles with >= GHOST_MAX DUs since their last flash get a flashing GC16 —
 * only there (the old "full screen every 25 cleanups" flashed the
 * panel and other windows every ~20-80 s while scrolling)
 *
 * - windows: a top-level window that maps (not popups/menus), unmaps, moves or
 * resizes (a popup closing, a workspace switch, tiling) reports its old/new area to the policy
 * (pol_window): what repaints there next gets one local flash at the idle
 * cleanup — the partial GC16 / DU that showed it leaves the ghost of the
 * content it replaced. Tracked via SubstructureNotify on the root: the direct
 * children are xfwm4's frames and the override-redirect popups/menus.
 * A popup/menu that maps (override-redirect) is painted fast instead:
 * dithered DU/A2 at once, the GC16 of its real greys queued behind it.
 *
 * FIFO commands (also env GLASSINE_EPDD="..." at start):
 * full full-screen GC16 now
 * verbose toggle the update log
 * du_max=F largest screen fraction sent as DU (default 0.03)
 * flash_min=F smallest screen fraction sent as flashing GC16 (default 0.35)
 * idle_ms=N the settle: GC16 cleanup after N ms without damage (default
 * 250, E-ink Center "Clean up after"; less = sooner after a stroke, but a pause mid-drawing
 * settles too)
 * pace=engine|fixed motion runs on the mainline engine: the next frame is read when
 * the engine can start it (default engine: what the panel takes,
 * no frame read only to be replaced in the queue) /
 * every run_ms
 * motion=binary|adaptive motion runs: one paint for every step (binary:
 * dithered A2/DU; the engine's default: adaptive mmin=mmax=a2_2,
 * i.e. fixed A2/2 with payback) / the paint follows the speed of the finger
 * or fling, GC16 .. DU4 .. A2 .. A2 cut to 8/6/4(/2) frames (engine
 * only; epd-policy.h "motion=adaptive")
 * ladder=a,b,c,d,e[,f] adaptive: the speeds (screen heights/s) between its seven
 * steps (default 0.12,0.3,0.8,1.6,3.2); f, into A2/2 (1 drive
 * frame, faint), is off unless given
 * steps=a,b,c[,d] adaptive: frames of the cut A2 steps (default 8,6,4,2;
 * 2..10, falling)
 * ahead=predict|run|off sway (wl), motion runs: when to answer a frame (DONE) so
 * sway and the app draw the next one. The frame is in wlsrc
 * once it arrived, so it may go early. predict (default):
 * when the next read is due within the time sway took to
 * deliver a frame after DONE (75th percentile of the last 16)
 * — the read is due when the engine frees the run's area
 * (epd_stream_free_ms: what runs there, by its real waveform
 * length) minus the read + hand-off lead; the next frame
 * then likely lands just before it. run: at once; off: only
 * after the read (one frame in flight, the apps at the
 * panel's pace)
 * ahead_ms=N ahead: at most one early DONE per N ms (default 23: <= 43.5 app frames/s, the A2/2 stagger shows a step every 2nd 85 Hz frame = 42.5/s; was 30)
 * ingest=gpu|cpu|check sway on the GPU (GLES, dma-bufs; ): its frames
 * turned into grey by a shader (epd-ingest.c, default gpu) /
 * by the CPU through the buffer's mapping
 * epd_ingest_frame_cpu, the GPU path's bytes, no ring job;
 * the rate depends on the exporter's caching -- msm GBM
 * buffers map write-combined: main_prof ing_read tells) /
 * both, the CPU result used and every
 * difference counted ("stats": ingest). pixman sway (shm)
 * is always the CPU way
 * ingest_kernel=row|tile|scalar ingest=cpu's convert : row =
 * today's (default; the WC-safe one), tile = R8 rows with
 * the LUT in registers / XRGB in 64 x 64 tiles + a NEON
 * transpose (for sway-heap's cached buffers), scalar = the
 * tile loop in plain C (the A/B). The same bytes. Also
 * GLASSINE_INGEST_KERNEL; live
 * ingest_bands=N ingest=cpu frames of >= 0.3 Mpx in N parts (1..4; default
 * 1) on the ingest's own pool (GLASSINE_INGEST_CPUS, default
 * 1-4); also GLASSINE_INGEST_BANDS; live
 * ingest_sync=off|on|N ingest=cpu's DMA_BUF_IOCTL_SYNC on a helper pinned to
 * CPU N (on = 0, a Silver core): START waited for, END
 * not (default off; also GLASSINE_INGEST_SYNC; the helper keeps
 * its first CPU). "stats" prints the three when one is on
 * ingest_partial=on|off ingest=cpu with a switch on: the syncs cover
 * only the bytes read (DMA_BUF_IOCTL_SYNC_PARTIAL, GlassineOS
 * linux 9fbb3532981d; a kernel without it: the whole buffer,
 * as off). Default on; also GLASSINE_INGEST_PARTIAL; live
 * run_ms=N motion runs, pace=fixed or /dev/ebc: at most one update per N ms
 * (default 100; stock: every SEND_UPDATE costs ~47 ms kernel CPU)
 * run_fold=N motion runs: DU updates per burst (default 1)
 * run_end=partial|flash end of a run: partial GC16 (~0.5 s, no flash) unless its area
 * is ghosted (then a local flash, ~1 s) / always the local flash
 * quiet_ms=N no input for quiet_after ms: at most one update per N ms (default
 * 2000, 0 = off) — clocks, htop, progress bars collect instead of
 * costing ~47 ms kernel CPU per repaint; any touch/pen/key lifts it
 * quiet_after=N ms without input before quiet_ms applies (default 5000)
 * jit=on|off engine: each drive frame rendered just before its flip
 * (default on) / as soon as the last one flipped
 * jit_size=on|off jit: the render lead by the next frame's size (pen piece,
 * mid, whole panel, stagger; epd-jit.h, default on: one big
 * frame no longer holds the small frames' lead high) / one
 * decaying max over every frame (the old lead)
 * handoff=inbox|lock engine: a submit is a message the drive thread applies
 * (default; the main/pen thread never waits for the stream
 * lock) / the submitter takes the stream lock itself (the
 * old path: 10-16 ms per hand-off on a scroll; )
 * drive_prof=on|off engine (default off): the drive thread's
 * split per loop iteration (drain, schedule + claim parts,
 * upload, GPU submit, fence wait, CPU render, advance, flip
 * wait, flip, JIT sleep, slack; by frame type), a summary
 * every prof_every s on stderr ("epd-prof drive/update/
 * threads/cores" lines; fields: epd-drm.c "drive_prof"),
 * end-to-end records per update, the threads' CPU/wait
 * (schedstat), core busy and CPU pressure from a non-RT
 * sampler; "stats" also writes the newest 1024 records to
 * /run/glassine-epdd-desktop-frames.csv. eink.conf: @drive_prof=on
 * drive_prof_csv=PATH|off ... plus one CSV row per iteration into PATH (and
 * PATH.freq: the 20 Hz cpufreq samples); implies on. PATH must be a file
 * name in /run/glassine-epdd-prof (the daemon creates it; root's, 0750,
 * the panel group, video by default; epd-profpath.h): any other path is refused, and the file is
 * created at 0640. The path must not contain another command's word (stats,
 * sleep, state, heal, mark, wake, ...): e.g. /run/glassine-epdd-prof/dprof.csv
 * main_prof=on|off the main thread's split per burst (snap/ingest, scan,
 * policy, dizzy, commit, the sends; "epd-prof main" lines)
 * main_prof_csv=PATH|off ... one CSV row per burst into PATH (the same rule)
 * prof_every=N both summaries every N s (default 5)
 * busy=F busy content (video, animations): >= 3 bursts a second covering
 * >= F of the screen count as motion like a finger, so their run
 * waits for the app's frame gap (default 0.03, 0 = off; see BUSY_N
 * in epd-policy.h). Per app: eink.conf busy=on|off
 * src=x|fb read pixels with XGetImage (default) or from /dev/fb0 (no cheaper:
 * uncached, see snap)
 * srccheck log whether fb0 and XGetImage agree on every pixel
 * shot save the panel buffer and a fresh full-screen read as
 * panel-order PGMs in /run/glassine-epdd-prof (glassine-shot-panel.pgm,
 * glassine-shot-src.pgm; dcmap and srccheck write theirs there too)
 * alpha_cut=F GPU compositing: ARGB pixels with alpha below F are left out
 * (soft shadows, rounded-corner fringes), the rest is opaque (0.5)
 * flash_full=F flashes of at least this screen fraction use the driver's
 * full-panel flash (faster), smaller ones flash locally (0.6)
 * burst=du old behaviour: every burst DU (+ idle cleanup); burst=auto: sized
 * clean=partial|full idle cleanup of DU areas as GC16 partial / full-mode (flash)
 * deadband=T sway's GPU keeps a pixel's last accepted grey while
 * |new - accepted| < T (0..16, default 0: off, no extra work;
 * a GLES grey sway only, others: nothing): fewer
 * dot flips in video and slow gradients, error < T. After a
 * quiet gap (idle_ms / 2) sway sends the true picture, which
 * the cleanup then reads. eink.conf: @deadband=T
 * gamma=G grey curve out = 255*(in/255)^G (default 1.8). The panel
 * shows 16 levels (top nibble), so UI light greys (Adwaita's
 * #f6f5f4, #deddda) land on white; G > 1 darkens them into
 * visible levels. Triggers a full refresh.
 * grey=luminance|luma sway frames' grey value (epd-grey.h): true luminance
 * (default, saturated colours keep their
 * lightness, pure green no longer level 6) or the old luma.
 * Sway's own grey output (R8) follows @grey at sway's start
 * (its HELLO says which); X stays luma (EOL)
 * ink=off|apps direct pen preview (below): never / in the ink areas: under sway
 * the visible part (nothing above it) of each window whose app has
 * ink=on (eink.conf), from sway's "ink" lines; under X the active
 * window's app
 * bench=start hand the panel to glassine-epd-bench for one synthetic test (it
 * execs back; see glassine-epd-bench.c: start it with glassine-epd-bench start)
 * park=on|off after a finger lifts, move the pointer onto an invisible
 * corner window (default off; see pointer parking below)
 * pointer=on|off sway: the mouse pointer on the panel (default on; see "the
 * mouse pointer" below) / never (one erase if it shows)
 * reexec exec /usr/sbin/glassine-epdd-desktop again (same PID: updates without
 * ending the desktop session)
 * temp=N|auto mainline engine: waveform temperature N C (A/B tests: the
 * battery is not the panel, e.g. warm while charging) /
 * the battery's again; from the next update
 * dry=on|off mainline engine: link without the panel's high voltage (the
 * panel does not change; timing diagnosis) / normal. Live from
 * the next link up, no reexec (so also after a GPU hang); env
 * GLASSINE_EPDD_HV = 0 / 1 keeps it across later reexecs
 * gpu=off|on|check mainline engine: drive frames on the CPU / on the GPU / on the
 * GPU with the CPU rendering too, every 16th frame read back and
 * compared (a difference sends the CPU's and turns the GPU off);
 * gpu=pad: the CPU renders into the GPU's scanout buffers
 * (diagnosis). Reexecs (env GLASSINE_EPDD_GPU = 0 / 1 / 2 or check /
 * 3 or pad); default off. the old check read the scanout
 * mapping (~450 ms a frame) and so over-drove (inverted) the
 * panel: new GPU paths get dry=on runs first
 * gpe=off|on|check mainline engine, with gpu=on: the engine's per-pixel state on the
 * GPU: claims, finishes and blocked tests there, the
 * drive frames read it / the same with the CPU doing every pass too,
 * each claim compared, every 64th the whole state (a difference turns
 * it off). Reexecs (env GLASSINE_EPDD_GPE = 0 / 1 / 2 or check); default
 * off. A GPU hang with the state on it: levels assumed white + a
 * full flash
 * gtarget=off|on|check with gpe=on and sway's dma-bufs through the ingest: the
 * update targets drawn on the GPU (stage 5) from a
 * mirror of the shadow there, no CPU copy per hand-off / the same,
 * every target read back and compared with the CPU's (a
 * difference turns it off). Reexecs (env GLASSINE_EPDD_GTARGET = 0 / 1 /
 * 2 or check); default off
 * reload re-read /etc/glassine/eink.conf (also done when it changes)
 * cut=auto|N DU frame cut (0..5: 22..17 frames at
 * 24 C, 267..210 ms; ). auto (default): 5 during motion runs,
 * 0 otherwise; only switched with no DU in flight (the driver
 * rebuilds the DU LUT in place)
 * matched=on|off motion runs' cut A2 steps: a pixel whose transition opposes
 * its DC record gets the length that cancels it (2..10 frames)
 * instead of the step's (epd_set_matched); default on (with
 * model=panel: the length the pixel needs, see model=)
 * matched_from=N matched_cap=N where it may lengthen: cut steps of >= N frames
 * only (default 2: every cut step), by <= N frames (default 3;
 * 0 = to any length, v1; 3 "made a hell of a
 * difference" over 0, paying pixels took new content sooner)
 * payback=on|off glassine-settings "Anti-ghosting": the defaults of matched=,
 * payback=, heal= and model= below / the paint alone
 * (matched=off payback=wait heal=0 model=dc snap=off)
 * payback=nonblock|preempt|wait matched cuts: pixels made longer than their step do
 * not hold up the next step (it starts around them; they take
 * the newest content when free; epd_set_nonblock). preempt:
 * as nonblock, and a paying pixel whose content changes takes
 * the new step at once (with model=panel). Default preempt
 * heal=N heal_from=N payback=nonblock: in scroll steps cut to >= heal_from frames
 * (default 2) or uncut, idle unchanged black/white pixels the
 * panel model finds >= N frames short of their level get a few
 * drive frames toward it, without holding up a step
 * (epd_set_heal; the leftovers of a fast scroll stayed while it
 * slowed down). Default 2; 0 = off
 * heal_len=N heal runs of up to N drive frames whatever the step's
 * length (A2/2 steps give 1, too few to heal; 0 = len - 1)
 * snap=on|off GC16/DU/DU4 start a black/white pixel from where the panel
 * model has it: one a cut A2 left nearer its target's rail
 * is not swung the whole way (+20 to the DC record per cycle)
 * and the heal tops up every idle pixel of the rect short of
 * its level (a partial GC16 skipped those: epd_set_snap).
 * Part of payback=on (default on)
 * model=dc|panel|optical|track matched cuts' lengths from the DC record or the
 * panel model (default: to what a pixel needs to reach its target,
 * longer where the DC record asks for more; optical: the model
 * alone, the record drifts); track:
 * keep the model only for the dc: stats' shortfall (epd_set_model)
 * claim_kernel=soa|packed|check the claim's pixel pass on the engine's six
 * arrays (soa, default) or on the unified state's packed
 * words D/W/T, one sweep per row (packed;
 * the same decisions bit for bit, the arrays still the
 * engine's state, each row packed and unpacked);
 * check: both, the arrays' result kept, differences
 * counted and logged (slow). Also GLASSINE_CLAIM_KERNEL; live
 * @claim_state=arrays|packed|check eink.conf only (or GLASSINE_CLAIM_STATE): the
 * engine's per-pixel state: today's six
 * arrays, the packed words D/W/T as the only state (W shared
 * with the GPU: no W upload; the daemon's default since
 * 10-29), or both compared per claim. NOT live: read when the engine opens (a reexec)
 * @stagger_state=lanes|runs eink.conf only (or GLASSINE_STAGGER_STATE): the A2
 * pass (stagger) on its own state, loaded at a run's start
 * and handed back at its end (runs), or on the engine's
 * D and T in place, cut steps armed per 16-px line and
 * evicted at the run's end (lanes, the default since 10-29,
 * /: no state crosses; needs
 * @claim_state=packed, else runs). NOT live: read when the engine opens
 * claim_bands=N claims of >= 0.3 Mpx in N row bands (1..4; default 4; 1 = one band,
 * the old way) on the worker pool, either kernel, the same bytes
 * . Also GLASSINE_CLAIM_BANDS; live. With N > 1 the
 * stream thread runs at FIFO 51 (above the workers)
 * drive_overlap=on|off big claims go out as a job (its N bands on the workers)
 * and take effect two frames later; the stream thread never
 * waits for one (default off; live, from the next loop)
 * update_job=pool|main a burst's scan, commit and the levels of its big sends
 * (>= 0.25 Mpx) in parts on a pool, the main thread taking
 * parts too and waiting (default pool; main =
 * the old one pass). The same bytes. Also GLASSINE_UPDATE_JOB; live
 * update_cpus=gold|silver|LIST where the job's helpers run (default silver
 * since 10-27, ; LIST like 1-4). gold with update_rt=fifo =
 * the stream's own frame workers (shared with claim_bands:
 * a worker takes band and render parts first); every other
 * choice = 4 helpers of the job's own. GLASSINE_UPDATE_CPUS; live
 * update_rt=fifo|nice fifo (default): the stream's workers at SCHED_FIFO 50 (the
 * stream thread then at 51, above them); the own pool's
 * helpers at FIFO 39, below the pen thread (40). nice:
 * SCHED_OTHER nice -10, as the main thread. GLASSINE_UPDATE_RT;
 * live (at the next job)
 * worker_rt=fifo|nice the stream's frame workers (render, copy, claim bands,
 * gold+fifo update parts): SCHED_FIFO 50 (default, today) or
 * SCHED_OTHER nice -10 (no rt_umin then); the stream thread
 * stays FIFO either way. GLASSINE_WORKER_RT; live (at the next job)
 * stagger=on|off motion runs' cut A2 steps (gpu=on, gpe=off): staggered on the
 * GPU instead of stepped whole: each pixel steps
 * on its own blue-noise phase group, 1/len of the area starts a
 * step each frame, so new content shows from the next frame
 * on; greys dithered by the same pattern. Ends at the run's
 * cleanup. Live; default off. stagger=check: on, and the CPU
 * reference steps a copy in lockstep (every 4th frame's words and
 * each hand-back's state compared, differences logged; slow)
 * drive_state=heap|tex the LUT drive shader's per-pixel codes :
 * heap = W in a shared dma-buf, written in place by the CPU and
 * sampled by the shader (no st upload; ABGR16161616, ABGR8888 if
 * that does not import); tex = the RGBA32UI texture st, packed
 * and uploaded (the default). The same frames. NOT live: taken
 * when the GPU opens (eink.conf "@drive_state=heap" at the
 * start, or GLASSINE_DRIVE_STATE=heap); a later change waits for
 * the next open (an epdd restart, or the GPU dropped and back)
 * stagger_direct=0|1 stagger frames image-stored into the scanout buffer (1, the
 * default) or drawn through its framebuffer as the engine's
 * frames are (0); live
 * stagger_tskip=on|off the stagger pass skips tiles (256 x 8 px) idle for a
 * group cycle: no work, no target or state change (their
 * no-drive output only; the same result). Default on
 * (606afac; not with stripes in grey mode); live
 * stagger_lead=on|off a workaround for the A509's lost wave :
 * every stagger compute pass gets a leading workgroup row
 * whose stores go to scratch rows. Default off (also
 * GLASSINE_STAG_LEAD=1); NOT live: on only if on when the
 * stagger's GPU side opens (eink.conf "@stagger_lead=on"
 * at the start), a later "on" waits for the next open
 * stagger_ctl=0|1|2|3|4 stagger_pay=N the stagger's controller: steps only as long
 * as the panel model needs to reach the rail / and DC payback
 * steps for records >= N frames (0 = none); 2: payback also
 * through the idle slots, same-way steps without an idle
 * between (epd-stagger.h); 3: a pulse instead of the idle
 * frame (a step may drive its whole cycle, reversals too).
 * Default ctl 3, pay 0
 * 4: ctl 3's steps on GPU bit planes (epd-bp4.h,
 * the bit planes): lanes, stagger_pay=panel, cap <= 3 and a 64-row
 * layout only (else ctl 3 runs, said once); one-frame
 * steps (G 2); its idle period is fixed (not stagger_k)
 * stagger_pay=panel stagger_cap=N ctl 1-3: panel mode's payback rule instead
 * (matched cuts): a pixel going to the other rail drives
 * the longer of its need and the record against it, at most
 * N frames past its need; nothing at an idle rail. N
 * default -1 = matched_cap (0: none = 7, the most it holds;
 * 1-7). stagger_pay=N goes back to the frames rule. Default
 * off (epd-stagger.h)
 * stagger_stripe=0|64|blocks|cells[N|WxH]|wbayer the phase groups as 64 px column stripes instead of
 * the blue-noise ranks (faster on the GPU:
 * 18.1 -> 15.9 ms a full scroll, measured in the since
 * removed grey mode). Default 0 = the blue
 * noise (64 was the earlier default).
 * stagger_stripe=blocks: the stripes
 * cut into 64 x 64 blocks, a block's group by a 16 x 16
 * Bayer pattern over the block grid (G 2: a checkerboard;
 * no long vertical seams, the same work; 2).
 * Default off. stagger_stripe=cells (8 x 8), cells16,
 * cells32, cellsWxH (W 8 16 32 64, H 1..64, powers of
 * two): W x H cells, a cell's group by its blue-noise
 * rank inside its 64 x 64 tile (a word one group; the
 * GPU runs a tile's cells in rank order: a wave one
 * group; ). Default off. stagger_stripe=wbayer:
 * 8 x 1 cells (a word) in a Bayer order over (word, row):
 * G 2 a checkerboard of words (looks like
 * blue noise mid-scroll, a word one group). Default off.
 * All live, from the next motion run
 * stagger_k=K ctl 3: the most pulses without an idle frame (default 8)
 * stagger_idle=pixel|shared ctl 4: the idle per pixel as ctl 3's K (pixel,
 * default) or the shared word-diagonal mask (shared: cheaper
 * on the GPU, 32 px columns idle together). From the next run
 * target_pool=N the update targets' buffers of 128 KB and more kept for
 * reuse, 0..8 (0 = off: malloc/free each, as before);
 * default 4. The same output; "stats" prints the hits
 * stagger_a21=on|off ctl 3: A2/1 -- an A2/2 run's steps at G 1 (every pixel
 * decides on every panel frame, a pulse a frame, an idle
 * after K) instead of G 2. Default off, next run
 * stagger_linger=MS it stays on MS after the targets last changed (default 250)
 * stagger_warm=on|off the next run's state work done once the panel rests
 * (the engine idle after its last update): its start then
 * finds nothing to resolve. Default on
 * stagger_entry=first|engine first (default): a scroll's first cut step goes
 * to the warm stagger, which starts at once (no engine cut,
 * no wait for it). engine: the engine does the first cut
 * hammer_top=N the rail hammer (stagger_pay=panel): an arrival at black /
 * white with a partial history (from a grey, or a reversal:
 * the model short of the whole drive) gets N frames (0..7)
 * past its need into the rail -- an optical reset (default
 * 2; 0 = off; next run)
 * gpu_hang_test=1 a test: the next GPU drive frame is not waited for and
 * taken as hung (the abandon), the revive brings the GPU
 * back once it finished (2 s later)
 * stagger_timer=0|1 GPU timer queries on the stagger's passes (stats: GPU ms)
 * dither=on|off motion runs: ordered-dither greys to black/white for DU (default
 * on) instead of leaving them for the cleanup; the run's end
 * GC16 restores real greys
 * popupdu=dither|bw popup=du's DU: greys ordered-dithered (default) or thresholded
 * to black/white (the GC16 behind swings only the true greys)
 * dither_pattern=blue|bayer|dizzy the dither of dither= / popupdu=dither (C_DITHER)
 * and DU4 (C_Q4), CPU and GPU alike: 64 x 64 blue noise
 * (default; transposed + shifted by 32, independent of the
 * stagger's groups), the old 8 x 8 Bayer, or dizzy (error
 * diffusion in a random order, epd-dizzy.h, CPU only: its
 * commits are uploaded, not mirrored on the GPU; C_Q4 and the
 * stock kernel's ebuf take blue noise). The stagger dithers
 * nothing. An app's own: dither= in its eink.conf
 * line. Live, from the next commit
 * dizzy_reuse=on|off dizzy: a scroll's dots move with the content (a tile
 * whose greys moved unchanged keeps its dots, the rest is
 * dithered; commit_dizzy). Default off (a feel check)
 * dizzy_where=gpu|cpu dizzy's dots on the GPU (the ingest's context, ES 3.1
 * compute; epd_ingest_dizzy) when the ingest is up, else
 * the CPU bands; motion runs and layered stills only (serial
 * stills and dizzy_reuse stay on the CPU). Default cpu: the
 * layered shader faults (barrier with >1 wave) or is 7x
 * slower on the A509; the block form (below)
 * is untested there
 * dizzy_form=block|layered which GPU shader (dizzy_where=gpu): block = one
 * invocation per 8x8 block, no shared memory or barriers,
 * the dots of epd_dizzy_block (not the CPU's layered dots:
 * dizzy-like), a wlsrc frame's rect read from
 * the ingest's own texture (no upload); layered = the
 * shader, = the CPU's dots. Default block. Live
 * dizzy_threads=N dizzy's threads, 1..8 (the main thread included;
 * default 8, ~15 % faster than 4). The 4th and later run on the slow cores
 * (the stream's SCHED_FIFO threads keep the fast ones).
 * The dots don't change with N (fixed 64-row bands). Live
 * dizzy_still=layered|serial dizzy outside motion runs (presses, popups):
 * the 8-layer form (default: the faster one) or the gist's
 * serial one (a little better near the rails, ~1.5x the time);
 * motion runs always layered
 * a2=on|off engine only: DU updates run as the WBF's A2 (10 frames, ~120 ms
 * at 24 C) where every changed pixel is black/white -> black/white,
 * else DU (default on)
 * boost=N any touch/pen/key input raises every CPU cluster's minimum clock
 * to its top until N ms after the last input (default 0 = off:
 * no gain measured, see boost_set)
 * boost_run=on|off with boost=N: the boost also lasts while a motion run does, up to
 * 3 s after the last input (default off; no measured
 * gain over boost=150, 2+2 windows; )
 * main_nice=N the main loop's (ingest + policy) nice, -20..19 (default -10), and
 * main_slice=US its EEVDF slice in us (sched_attr.sched_runtime, 100..100000; 0 =
 * the kernel's 2.8 ms; default 1000). A device A/B (GNOME
 * Software scroll + 8 busy loops in the desktop's cgroup):
 * late frames per 30 s 13/15/19 -> 7/13/10; the slice alone did
 * nothing, the weight does. Nothing changes with idle CPUs. Only
 * this thread: the SCHED_FIFO stream/pen threads keep theirs, and
 * epdd starts no apps (sway would pass a nice on to every app)
 * nwork=N engine only: threads a frame's render/copy is split over, 1..4
 * (default 4; )
 * rt_umin=N|default engine only: uclamp.min (0..1024) of those SCHED_FIFO threads;
 * default = the kernel's RT default (1024: RT runs at fmax)
 * qos=global|off|cpu:N engine only, from the next link-up: CPU wakeup latency hold
 * while the link is up. global (default) = cpu_dma_latency 0, all
 * CPUs WFI only; cpu:N = only the stream's CPUs, N us (cpu:0 =
 * none; fast-cluster core pc exits in 563 us, the budget ~700 us)
 * keepalive=N engine only: link + rails stay up N ms (<= 5000) after input, so
 * the next update starts at once (default 2000; 0 = off)
 * link_warm=N engine only: after the 400 ms linger the rails go down but the
 * DSI pipe + bridge stay up N ms ("warm idle"), so the next update
 * or pen hover only raises the rails (kernel 0051 "rails" param;
 * without it: the old full link down). Default 30000; 0 = off.
 * sleep/pause park the stream: the link goes fully down at once
 * link_log=on|off engine only: log every link transition (cold / warm up, warm,
 * down) with its CLOCK_MONOTONIC time and durations (default off)
 * sleep / wake glassine-powerd around suspend: "SLEEPING" banner, updates off /
 * banner replaced by the screen, updates on
 * pause / resume stop touching the panel (e.g. while glassine-greytest owns it);
 * resume repaints everything with a full GC16
 * mark log the time (ms, the verbose log's clock): t0 for latency tests
 * impostor X Y W H FILE show these pixels now, before X has them (see impostor)
 * drag start|move|end ... a tile handle dragged: glassine-epdd-desktop paints it (see drag_cmd)
 * The driver queues and re-runs colliding updates itself (upd_collision_list
 * in its debug log), so updates are fire-and-forget.
 * Only ioctl 0x700c is used; unknown /dev/ebc ioctls panic this kernel.
 *
 * usage: DISPLAY=:0 glassine-epdd-desktop [-v]
 */
#define _GNU_SOURCE             /* CPU affinity (dizzy's slow-core workers) */
#include <dirent.h>
#include <stdarg.h>
#include <errno.h>
#include <sys/wait.h>
#include <linux/input.h>
#include <fcntl.h>
#include <math.h>
#include <linux/fb.h>
#include <poll.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>
#include <ctype.h>
#include <grp.h>
#include <X11/extensions/Xdamage.h>
#include <X11/extensions/Xfixes.h>
#include <X11/extensions/XInput2.h>
#include <X11/extensions/XShm.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <signal.h>
#include <pthread.h>
#include <sys/syscall.h>
#include "epd-policy.h"
#include "epd-motion.h"
#include "epd-drm.h"
#include "epd-waveform.h"
#include "epd-upd.h"
#include "epd-stagger.h"
#include "epd-comp.h"
#include "epd-glyphs.h"
#include "epd-wl.h"
#include "epd-ingest.h"
#include "epd-grey.h"
#include "glassine-epd-output-proto.h"
#include "epd-dither.h"
#include "epd-dizzy.h"
#include "epd-brush.h"
#include "epd-profpath.h"
#include "epd-group.h"
#include "epd-knobs-ctl.h"
#ifdef EPD_GPU
extern int epd_gpu_stag_timer;         /* epd-gpu.c: the stagger's timer queries */
extern double epd_gpu_stag_ms[3];
extern double epd_gpu_bp4_ms[4];         /* epd-gpu.c: stagger_ctl 4's pack, visit, drive, frames */
extern int epd_gpu_stag_tskip;         /* epd-gpu.c: stagger_tskip= */
extern int epd_gpu_stag_lead;          /* epd-gpu.c: stagger_lead= */
extern int epd_gpu_drive_state;        /* epd-gpu.c: drive_state= */
#endif

#define EBC_SEND_UPDATE 0x700c
#define PW 1872                 /* panel (landscape) */
#define PH 1404
#define CTL "/run/glassine-epdd-desktop.ctl"

struct onyx_epdc_update {
    int32_t y, x, width, height;
    int32_t waveform_mode, update_mode, marker, dither;
    uint32_t flags;
    int32_t reserved;
};

static int ebc = -1, verbose;
/* mainline backend: no /dev/ebc and no ebuf (NULL): the engine reads its
 * pixels straight from `shadow` (which is what an update shows, see commit)
 * and copies them at submit, like SEND_UPDATE does (was an XRGB
 * ebuf in RAM, copied to a grey buffer per update) */
static epd_stream *eng;
static int64_t eng_temp_ms;
static int temp_override = -100;     /* "temp=N": waveform temperature in C; -100 = the battery's */
static int ingest_mode = 1;         /* ingest=cpu|gpu|check (0/1/2), see wl_dma_frame */
static int ingest_wait = 1;         /* ingwait=on|off: wait for the buffer's producer on the CPU first (the split) */
static int ingest_view = 1;         /* ingest_view=on|off: read sway's tagged R8 frames in place (ingest=cpu; no copy) */
static void wl_view_off(void);
static void dz_prep_drop(void);
static int wl_lazy = 1;             /* lazy=on|off: convert a frame only when it is read (epd_wl_set_damage) */
static int band_t;                  /* deadband=T (0..16; sway's GPU) */
static void band_push(void);
static void wl_lazy_set(int on);
static int wl_ahead = 2, wl_ahead_ms = 23;  /* ahead=off|run|predict (0/1/2), ahead_ms= (sway: early DONE in runs) */
static box last_rd;                         /* the last burst's damage (portrait): the run's area */
static int64_t wl_done_at;
static int eng_a2 = 1;              /* a2=on|off: DU updates as A2 where all black/white */
/* payback for every cut A2 (user, "panel mode is orthogonal to it
 * all and should always be active for anything A2"): matched cuts with the
 * panel model's DC-aware lengths, preemption and healing; the knobs stay for
 * comparisons (matched=off payback=wait heal=0 model=dc = the paint alone) */
static int eng_matched = 1;          /* matched=on|off: matched cuts (epd_set_matched) */
static int eng_matched_from = 2, eng_matched_cap = 3;  /* matched_from= / matched_cap= (epd_set_matched_rule) */
static int eng_nonblock = 2;        /* payback=nonblock|preempt|wait (epd_set_nonblock 1|2|0) */
static int eng_heal = 2, eng_heal_from = 2; /* heal= / heal_from= (epd_set_heal) */
static int eng_heal_len;                    /* heal_len= (epd_set_heal_len) */
static int eng_snap = 1;                    /* snap=on|off (epd_set_snap) */
static int eng_model = 2, eng_track;    /* model=panel / optical / track (epd_set_model) */
static int dither_pattern = EPD_DITHER_BLUE;    /* dither_pattern=blue|bayer|dizzy (dither_table, commit_dizzy) */
static int dizzy_still_serial;                  /* dizzy_still=serial|layered */
static int dizzy_gpu = 0;                       /* dizzy_where=gpu|cpu (not "dizzy_gpu=": "gpu=" reexecs) */
static int dizzy_form_layered = 0;              /* dizzy_form=block|layered: the GPU shader (epd_ingest_dizzy_form) */
static int eng_keepalive_ms = 2000; /* keepalive=N: link + rails stay up N ms after input */
static int boost_ms;                /* boost=N: CPU clocks up to N ms after input, see boost_set */
static int boost_run;               /* boost_run=on: the boost lasts while a motion run does, see boost_tick */
#define ENG_LEVELS "/run/glassine-epdd-desktop.levels"
static volatile sig_atomic_t quit_sig;
static int ink_mode = 1;            /* 0 off, 1 the ink areas (sway: the ink=on apps' visible windows) */
static int ptr_mode = 1;            /* pointer=on|off (sway: the mouse pointer, see ptr_cb) */
static policy pol;
static double flash_full = 0.6;     /* see send */
static int src_fb = 0;               /* pixels from /dev/fb0 (1) or XGetImage (0), see snap */
static uint8_t lut[256];            /* luma's gamma LUT (X, the compositor, luma R8 frames) */
static uint8_t lut_lum[256];        /* luminance's (epd_grey_lum_lut) */
static int grey_lum = 1;            /* grey=: sway's XRGB frames by luminance */
static comp *cm;                    /* compositor (NULL: read the root) */
static uint32_t *ebuf;              /* /dev/ebc's buffer; NULL on mainline (engine) */
static uint8_t shadow[PW * PH];     /* what the panel shows (grey), panel coords */
static int marker;
/* ebuf, shadow and every update sent: the main loop and the pen thread
 * both write them. Held only while pixels are copied/drawn and updates
 * submitted — never across an X round trip (a snapshot takes ~100 ms). */
static pthread_mutex_t buf_mu = PTHREAD_MUTEX_INITIALIZER;

static void ingest_lut(void);
static void set_gamma(double g)
{
    if (!(g > 0.2 && g < 5)) g = 1.0;
    epd_grey_luma_lut(lut, g);
    epd_grey_lum_lut(lut_lum, g);
    if (cm) comp_set_lut(cm, lut);
    ingest_lut();
    fprintf(stderr, "glassine-epdd-desktop: gamma %.2f\n", g);
}

/* an app's own run range (eink.conf "paint=", glassine-settings' per-app
 * responsiveness): the burst's profile sets pol.m_min/m_max over the global
 * ones, kept here while it is in force (-1: none); the next burst with another
 * profile, or any setting, puts the global ones back first. The app's range
 * is still=range: its stills paint at its low end, LO == HI = one mode.
 * curve: the app's "curve=" (Gradual speed-up), -1 = the global one;
 * it goes with paint= (glassine-settings writes both), so it applies with a range */
static int glob_mmin = -1, glob_mmax = -1, glob_still, glob_curve;
static void run_range(int lo, int hi, int curve)
{
    if (glob_mmin >= 0) {
        pol.m_min = glob_mmin; pol.m_max = glob_mmax; pol.still_range = glob_still; pol.m_curve = glob_curve;
        glob_mmin = glob_mmax = -1;
    }
    if (lo >= 0) {          /* an app's range covers its stills too (still=range: its low end) */
        glob_mmin = pol.m_min; glob_mmax = pol.m_max; glob_still = pol.still_range; glob_curve = pol.m_curve;
        pol.m_min = lo; pol.m_max = hi; pol.still_range = 1;
        if (curve >= 0) pol.m_curve = curve;
    }
}

static int dz_reuse;                    /* dizzy_reuse (see dz_take) */
static uint8_t *dz_pg, *dz_pd;
static void ink_halo_set(int on);
static int dz_set_threads(int n);      /* (dizzy_threads=, after dzj below) */
/* main_nice= / main_slice=: the main loop's weight and slice (see the list above).
 * sched_setattr keeps SCHED_OTHER; threads the main thread creates later inherit both */
static int main_tid, main_nice = -10, main_slice_us = 1000;
struct main_sattr { uint32_t size, policy; uint64_t flags; int32_t nice; uint32_t prio; uint64_t runtime, deadline, period; uint32_t umin, umax; };
static void main_sched_apply(void)
{
    if (!main_tid) return;
    struct main_sattr a = { .size = sizeof a, .policy = 0, .nice = main_nice, .runtime = main_slice_us * 1000ull };
    if (syscall(SYS_sched_setattr, main_tid, &a, 0))
        fprintf(stderr, "glassine-epdd-desktop: main_nice=%d main_slice=%d: %s\n", main_nice, main_slice_us, strerror(errno));
    else fprintf(stderr, "glassine-epdd-desktop: main loop nice %d, slice %s%d us\n", main_nice, main_slice_us ? "" : "default ", main_slice_us);
}
/* settings: the policy's, plus gamma and pen ink (daemon side) */
/* main_prof (default off): the main thread's split
 * per burst (one read of the pending damage: snap, scan, commit, the sends).
 * Types: burst-run (the policy in a motion run) / burst-still. Fields (ms
 * unless named otherwise; the ingest ones = every frame ingested since the
 * last burst, at dispatch or in the burst's snap when lazy):
 * snap the reads (sway: epd_wl_flush, so the lazy ingest is in here)
 * ing_wait poll on the dma-buf's write fence (ingest=cpu: + SYNC START) ing_gpu the ingest's draw + fence (cpu: 0)
 * ing_read its copy out (memcpy from the cached target / glReadPixels; cpu: the convert from the mapping)
 * ing_n frames ingested (count) conv the CPU convert of shm frames (wl_rect / wl_grey)
 * scan scan shift measure_shift (the motion estimate)
 * policy pol_step + the profile lookup + pol_burst dizzy dz_prep (dizzy's dots before the lock)
 * commit the commit loop (incl. dizzy's locked part)
 * send run_actions after the commit (the sends: target + hand-off)
 * target ... of which epd_target_make append ... the inbox append (or the stream lock)
 * subs submits (count) total the burst, start to end
 * Summary every prof_every s on stderr ("epd-prof main ..."); main_prof_csv=PATH: one row per burst.
 * With drive_prof on the sends carry end-to-end stamps (epd_stream_set_origin): the burst's
 * first damage, the last fence seen, ingest done, scan done, commit done. */
#include "epd-prof.h"
enum { MT_RUN, MT_STILL, MT_N };
static const char *const mt_name[MT_N] = { "burst-run", "burst-still" };
enum { MF_SNAP, MF_ING_WAIT, MF_ING_GPU, MF_ING_READ, MF_ING_N, MF_CONV, MF_SCAN, MF_SHIFT, MF_POLICY, MF_DIZZY,
       MF_COMMIT, MF_SEND, MF_TARGET, MF_APPEND, MF_SUBS, MF_TOTAL, MF_N };
static const char *const mf_name[MF_N] = { "snap", "ing_wait", "ing_gpu", "ing_read", "ing_n", "conv", "scan", "shift",
                                           "policy", "dizzy", "commit", "send", "target", "append", "subs", "total" };
static int main_prof;                   /* main_prof=on|off */
static epd_prof *mprof;                 /* (allocated at the first on) */
static float mcur[MF_N];                /* the burst being recorded */
static FILE *mprof_csv;
static char mprof_csv_path[256];
static int prof_every = 5;              /* prof_every=N s: both summaries */
static int drive_prof = -1;             /* drive_prof=on|off (-1: never said) */
static char drive_prof_csv[256];        /* drive_prof_csv=PATH|off: the last one sent */
static int drive_prof_csv_new;          /* ... not sent yet */
static double mo_fence_t, mo_ing_t;     /* (drive_prof) the last ingest's fence seen / done */
#define MP_ON __builtin_expect(main_prof, 0)
#define MADD(f, v) do { if (MP_ON) mcur[f] += (float)(v); } while (0)
static pthread_t main_thread;           /* (set in main: eng_update counts only its submits) */

static void main_prof_print(void)
{
    if (!mprof) return;
    epd_prof_print(mprof, stderr, "main", NULL);
    if (mprof_csv) fflush(mprof_csv);
}

static void main_prof_close(int type, double t0)
{
    epd_prof_add(mprof, type, mcur);
    if (mprof_csv) epd_prof_csv_row(mprof, mprof_csv, t0, type, NULL, mcur);
    memset(mcur, 0, sizeof mcur);
}

static void main_prof_set(int on)
{
    if (on && !main_prof) {
        if (!mprof && !(mprof = calloc(1, sizeof *mprof))) return;
        epd_prof_init(mprof, MT_N, mt_name, MF_N, mf_name);
        memset(mcur, 0, sizeof mcur);
        main_prof = 1;
        fprintf(stderr, "glassine-epdd-desktop: main_prof on (every %d s)\n", prof_every);
    } else if (!on && main_prof) {
        main_prof_print();
        main_prof = 0;
        fprintf(stderr, "glassine-epdd-desktop: main_prof off\n");
    }
}

/* a ctl value: up to the next blank */
static void prof_token(char *d, size_t n, const char *v)
{
    size_t k = 0;
    while (v[k] && v[k] != ' ' && v[k] != '\t' && v[k] != '\n' && k + 1 < n) { d[k] = v[k]; k++; }
    d[k] = 0;
}

/* a file the daemon names itself (shot, dcmap, srccheck): in EPD_PROF_DIR, 0640,
 * never through a symlink; NULL (said on stderr) when it can't be had */
static FILE *prof_out(const char *name)
{
    char p[PATH_MAX];
    FILE *f = epd_prof_fopen(epd_prof_name(p, sizeof p, name));
    if (!f) fprintf(stderr, "glassine-epdd-desktop: %s: %s\n", p, strerror(errno));
    return f;
}

static void main_prof_csv_set(const char *v)
{
    char path[256];
    prof_token(path, sizeof path, v);
    if (!strcmp(path, mprof_csv_path)) return;      /* (an eink.conf reload: not truncated again) */
    int off = !*path || !strcmp(path, "off");
    if (!off && !epd_prof_path_ok(EPD_PROF_DIR, path)) {   /* (the FIFO and eink.conf are not trusted with a path) */
        fprintf(stderr, "glassine-epdd-desktop: main_prof_csv=%s: refused, a file name in %s only\n", path, EPD_PROF_DIR);
        return;
    }
    snprintf(mprof_csv_path, sizeof mprof_csv_path, "%s", path);
    if (mprof_csv) { fclose(mprof_csv); mprof_csv = NULL; }
    if (off) return;
    if (!mprof) main_prof_set(1);
    if (!mprof) return;
    if (!(mprof_csv = epd_prof_fopen(path))) { perror(path); return; }
    epd_prof_csv_head(mprof, mprof_csv, NULL);
}

/* drive_prof's settings into the stream (once it is open) */
static void drive_prof_apply(void)
{
    if (!eng || (drive_prof < 0 && !drive_prof_csv_new)) return;
    epd_stream_prof(eng, drive_prof, prof_every, drive_prof_csv_new ? drive_prof_csv : NULL);
    drive_prof_csv_new = 0;
}

/* live settings across a reexec (stagger_stripe=0 is a
 * live-only setting keeps, and every reexec dropped it): each
 * key=value of a ctl line is kept (the newest per key, in order), handed to the
 * new image in GLASSINE_EPDD_LIVE and applied after its first eink.conf. Not kept:
 * what reexecs or runs once by itself (gpu= gpe= gtarget= bench= src=
 * gpu_hang_test=, any =check) and the CSV paths. */
#define LIVE_MAX 64
static char live_kv[LIVE_MAX][96];
static int live_n;
static void live_note(const char *line)
{
    static const char *skip[] = { "gpu", "gpe", "gtarget", "bench", "src", "gpu_hang_test", "drive_prof_csv", "main_prof_csv" };
    const char *t = line;
    while (*t) {
        while (*t == ' ' || *t == '\t' || *t == '\n') t++;
        size_t n = strcspn(t, " \t\n");
        const char *eq = memchr(t, '=', n);
        if (n && n < sizeof live_kv[0] && eq && eq > t && strncmp(eq + 1, "check", 5)) {
            size_t kl = (size_t)(eq - t);
            int ok = 1;
            for (size_t i = 0; i < sizeof skip / sizeof *skip; i++) ok &= strlen(skip[i]) != kl || strncmp(t, skip[i], kl);
            for (size_t i = 0; i < kl; i++) ok &= isalnum((unsigned char)t[i]) || t[i] == '_';
            if (ok) {
                for (int i = 0; i < live_n; i++)
                    if (!strncmp(live_kv[i], t, kl + 1)) {         /* (key and '=') */
                        memmove(live_kv[i], live_kv[i + 1], sizeof live_kv[0] * (size_t)(live_n - i - 1));
                        live_n--;
                        break;
                    }
                if (live_n == LIVE_MAX) { memmove(live_kv[0], live_kv[1], sizeof live_kv[0] * (LIVE_MAX - 1)); live_n--; }
                memcpy(live_kv[live_n], t, n);
                live_kv[live_n++][n] = 0;
            }
        }
        t += n;
    }
}
static void live_export(void)
{
    char b[LIVE_MAX * 97] = "";
    size_t o = 0;
    for (int i = 0; i < live_n; i++) o += (size_t)snprintf(b + o, sizeof b - o, "%s%s", o ? " " : "", live_kv[i]);
    if (live_n) setenv("GLASSINE_EPDD_LIVE", b, 1); else unsetenv("GLASSINE_EPDD_LIVE");
}
/* update_job / update_cpus / update_rt, applied by
 * upd_apply (after commit, where the job's runner is) */
static int commit_skip;
static int update_job = 1;              /* 0 main, 1 pool (default since 10-26: the device A/B) */
static int update_rt;                   /* 0 fifo, 1 nice */
static int update_cpus = 1;             /* 0 gold, 1 silver (default since 10-27: Gold + the fg-boosted app contend), 2 update_list */
static char update_list[64];
static void upd_apply(void);
static void upd_parse(const char *cmd)
{
    const char *v;
    if (strstr(cmd, "update_job=pool")) update_job = 1;
    if (strstr(cmd, "update_job=main")) update_job = 0;
    if (strstr(cmd, "update_rt=fifo")) update_rt = 0;
    if (strstr(cmd, "update_rt=nice")) update_rt = 1;
    if ((v = strstr(cmd, "update_cpus="))) {
        v += 12;
        if (!strncmp(v, "gold", 4)) update_cpus = 0;
        else if (!strncmp(v, "silver", 6)) update_cpus = 1;
        else {
            size_t n = strcspn(v, " \t\n,;");
            while (v[n] == ',' && v[n + 1] >= '0' && v[n + 1] <= '9') n += 1 + strcspn(v + n + 1, " \t\n,;");
            if (n && n < sizeof update_list) { memcpy(update_list, v, n); update_list[n] = 0; update_cpus = 2; }
        }
    }
}

static void apply_cmd(const char *cmd)
{
    const char *v;
    /* the engine's, the drive's, the GPU's and the ingest's knobs: the list
     * (epd-knobs.h) and its parser (epd-knobs-ctl.h); the rest are the daemon's */
    epd_knobs_ctl(cmd, NULL, eng);
    if ((v = strstr(cmd, "main_nice=")) || strstr(cmd, "main_slice=")) {
        if (v) { int n = atoi(v + 10); if (n >= -20 && n <= 19) main_nice = n; }
        if ((v = strstr(cmd, "main_slice="))) { int u = atoi(v + 11); if (u == 0 || (u >= 100 && u <= 100000)) main_slice_us = u; }
        main_sched_apply();
    }
    run_range(-1, -1, -1);          /* mmin=/mmax= set the global range */
    pol_config(&pol, cmd);
    if ((v = strstr(cmd, "gamma="))) set_gamma(atof(v + 6));
    if ((v = strstr(cmd, "deadband="))) { band_t = atoi(v + 9); band_t = band_t < 0 ? 0 : band_t > GLASSINE_EPD_BAND_MAX ? GLASSINE_EPD_BAND_MAX : band_t; }
    band_push();                    /* (T, or idle_ms: the quiet gap follows it) */
    if (strstr(cmd, "grey=luminance")) { grey_lum = 1; ingest_lut(); }
    if (strstr(cmd, "grey=luma")) { grey_lum = 0; ingest_lut(); }
    if ((v = strstr(cmd, "flash_full="))) flash_full = atof(v + 11);
    if (strstr(cmd, "ink=off")) ink_mode = 0;
    if (strstr(cmd, "ink=apps")) ink_mode = 1;
    if (strstr(cmd, "ink_halo=on")) ink_halo_set(1);
    if (strstr(cmd, "ink_halo=off")) ink_halo_set(0);
    if (strstr(cmd, "ingest=cpu")) ingest_mode = 0;
    if (strstr(cmd, "ingest=gpu")) ingest_mode = 1;
    if (strstr(cmd, "ingest=check")) ingest_mode = 2;
    if (strstr(cmd, "ingest_view=on")) ingest_view = 1;
    if (strstr(cmd, "ingest_view=off")) { ingest_view = 0; wl_view_off(); }
    if (strstr(cmd, "ingwait=on")) ingest_wait = 1;
    if (strstr(cmd, "ingwait=off")) ingest_wait = 0;
    if (strstr(cmd, "lazy=on")) wl_lazy_set(1);
    if (strstr(cmd, "lazy=off")) wl_lazy_set(0);
    if (strstr(cmd, "ahead=run")) wl_ahead = 1;
    if (strstr(cmd, "ahead=predict")) wl_ahead = 2;
    if (strstr(cmd, "ahead=off")) wl_ahead = 0;
    if ((v = strstr(cmd, "ahead_ms="))) { wl_ahead_ms = atoi(v + 9); if (wl_ahead_ms < 0) wl_ahead_ms = 0; }
    if (strstr(cmd, "a2=on")) eng_a2 = 1;
    if (strstr(cmd, "a2=off")) eng_a2 = 0;
    if ((v = strstr(cmd, "keepalive="))) eng_keepalive_ms = atoi(v + 10);
    if ((v = strstr(cmd, "boost="))) boost_ms = atoi(v + 6);
    if (strstr(cmd, "boost_run=on")) boost_run = 1;
    if (strstr(cmd, "boost_run=off")) boost_run = 0;
    if (strstr(cmd, "pointer=on")) ptr_mode = 1;
    if (strstr(cmd, "pointer=off")) ptr_mode = 0;
    if (strstr(cmd, "src=fb")) src_fb = 1;
    if (strstr(cmd, "src=x")) src_fb = 0;
    if (cm && (v = strstr(cmd, "alpha_cut="))) comp_set_alpha_cut(cm, (float)atof(v + 10));
    if ((v = strstr(cmd, "matched_from="))) eng_matched_from = atoi(v + 13);
    if ((v = strstr(cmd, "matched_cap="))) eng_matched_cap = atoi(v + 12);
    if (eng) epd_stream_set_matched_rule(eng, eng_matched_from, eng_matched_cap);
    /* payback=on|off (glassine-settings "Anti-ghosting"): the defaults below / the
     * paint alone, the knobs after it on the same line still apply */
    if (strstr(cmd, "payback=off")) { eng_matched = 0; eng_nonblock = 0; eng_heal = 0; eng_model = eng_track = 0; eng_snap = 0; }
    if (strstr(cmd, "payback=on")) { eng_matched = 1; eng_nonblock = 2; eng_heal = 2; eng_heal_from = 2; eng_model = 2; eng_track = 0; eng_snap = 1; }
    if ((strstr(cmd, "payback=off") || strstr(cmd, "payback=on")) && eng) {
        epd_stream_set_matched(eng, eng_matched);
        epd_stream_set_nonblock(eng, eng_nonblock);
        epd_stream_set_heal(eng, eng_heal, eng_heal_from);
        epd_stream_set_model(eng, eng_model, eng_track);
        epd_stream_set_snap(eng, eng_snap);
    }
    if (strstr(cmd, "snap=on")) { eng_snap = 1; if (eng) epd_stream_set_snap(eng, 1); }
    if (strstr(cmd, "snap=off")) { eng_snap = 0; if (eng) epd_stream_set_snap(eng, 0); }
    if (strstr(cmd, "payback=nonblock")) { eng_nonblock = 1; if (eng) epd_stream_set_nonblock(eng, 1); }
    if (strstr(cmd, "payback=preempt")) { eng_nonblock = 2; if (eng) epd_stream_set_nonblock(eng, 2); }
    if (strstr(cmd, "payback=wait")) { eng_nonblock = 0; if (eng) epd_stream_set_nonblock(eng, 0); }
    if (strstr(cmd, "matched=on")) { eng_matched = 1; if (eng) epd_stream_set_matched(eng, 1); }
    if (strstr(cmd, "matched=off")) { eng_matched = 0; if (eng) epd_stream_set_matched(eng, 0); }
    if ((v = strstr(cmd, "heal="))) eng_heal = atoi(v + 5);
    if ((v = strstr(cmd, "heal_from="))) eng_heal_from = atoi(v + 10);
    if ((v = strstr(cmd, "heal_len="))) eng_heal_len = atoi(v + 9);
    if (strstr(cmd, "heal")) { if (eng) { epd_stream_set_heal(eng, eng_heal, eng_heal_from); epd_stream_set_heal_len(eng, eng_heal_len); } }
    if (strstr(cmd, "model=dc")) eng_model = eng_track = 0;
    if (strstr(cmd, "model=panel")) eng_model = 2;
    if (strstr(cmd, "model=optical")) eng_model = 1;
    if (strstr(cmd, "model=track")) { eng_model = 0; eng_track = 1; }
    if (strstr(cmd, "model=") && eng) epd_stream_set_model(eng, eng_model, eng_track);
    /* the update job */
    if ((v = strstr(cmd, "commit_skip="))) __atomic_store_n(&commit_skip, v[12] != '0', __ATOMIC_RELAXED);
    if ((v = strstr(cmd, "update_acct="))) __atomic_store_n(&upd_acct, v[12] == '1', __ATOMIC_RELAXED);     /* (step 3) */
    if (strstr(cmd, "update_job=") || strstr(cmd, "update_cpus=") || strstr(cmd, "update_rt=") || strstr(cmd, "worker_rt=")) {
        upd_parse(cmd);
        upd_apply();
    }
    if ((v = strstr(cmd, "dither_pattern="))) {
        dither_pattern = epd_dither_parse(v + 15);
        fprintf(stderr, "glassine-epdd-desktop: dither_pattern=%s (from the next commit)\n", epd_dither_name(dither_pattern));
    }
    if ((v = strstr(cmd, "dizzy_reuse="))) {
        dz_reuse = !strncmp(v + 12, "on", 2);
        fprintf(stderr, "glassine-epdd-desktop: dizzy_reuse=%s\n", dz_reuse ? "on" : "off");
    }
    if ((v = strstr(cmd, "dizzy_where="))) {
        dizzy_gpu = !strncmp(v + 12, "gpu", 3);
        fprintf(stderr, "glassine-epdd-desktop: dizzy_where=%s\n", dizzy_gpu ? "gpu" : "cpu");
    }
    if ((v = strstr(cmd, "dizzy_form="))) {
        dizzy_form_layered = !strncmp(v + 11, "layered", 7);
        fprintf(stderr, "glassine-epdd-desktop: dizzy_form=%s\n", dizzy_form_layered ? "layered" : "block");
    }
    if ((v = strstr(cmd, "dizzy_threads="))) {
        fprintf(stderr, "glassine-epdd-desktop: dizzy_threads=%d\n", dz_set_threads(atoi(v + 14)));
    }
    if ((v = strstr(cmd, "dizzy_still="))) {
        dizzy_still_serial = !strncmp(v + 12, "serial", 6);
        fprintf(stderr, "glassine-epdd-desktop: dizzy_still=%s\n", dizzy_still_serial ? "serial" : "layered");
    }
    if (strstr(cmd, "stagger"))
        fprintf(stderr, "glassine-epdd-desktop: stagger=%s ctl %d%s pay %s%d cap %d k %d linger %d ms direct %s stripe %d%s%.0d%s%.0d (needs gpu=on, gpe=off; from the next motion run)\n",
                epd_stream_stagger ? epd_stream_stag_check ? "check" : "on" : "off", epd_stream_stag_ctl,
                epd_stream_stag_a21 ? epd_stream_stag_ctl == 3 ? " A2/1" : " (a21 needs ctl 3)" : "",
                epd_stream_stag_payrule ? "panel, frames " : "", epd_stream_stag_pay, epd_stream_stag_cap, epd_stream_stag_k, epd_stream_stag_linger,
                epd_stream_stag_direct < 0 ? "default" : epd_stream_stag_direct ? "on" : "off", epd_stream_stag_stripe,
                epd_stream_stag_cellord ? " wbayer " : epd_stream_stag_cellw ? " cells " : epd_stream_stag_blocks ? " blocks" : "", epd_stream_stag_cellw,
                epd_stream_stag_cellw ? "x" : "", epd_stream_stag_cellh);
    /* the hand-off inbox (a device A/B: lock = the old path) */
    if (strstr(cmd, "handoff=")) fprintf(stderr, "glassine-epdd-desktop: handoff=%s\n", epd_stream_inbox ? "inbox" : "lock");
    /* the timing records (default off): drive_prof / main_prof */
    if ((v = strstr(cmd, "prof_every="))) { int n = atoi(v + 11); if (n >= 1 && n <= 3600) prof_every = n; }
    if (strstr(cmd, "main_prof=on")) main_prof_set(1);
    if (strstr(cmd, "main_prof=off")) main_prof_set(0);
    if ((v = strstr(cmd, "main_prof_csv="))) main_prof_csv_set(v + 14);
    if (strstr(cmd, "drive_prof=on")) drive_prof = 1;
    if (strstr(cmd, "drive_prof=off")) drive_prof = 0;
    if ((v = strstr(cmd, "drive_prof_csv="))) {
        char path[256];
        prof_token(path, sizeof path, v + 15);
        if (*path && strcmp(path, "off") && !epd_prof_path_ok(EPD_PROF_DIR, path))
            fprintf(stderr, "glassine-epdd-desktop: drive_prof_csv=%s: refused, a file name in %s only\n", path, EPD_PROF_DIR);
        else if (strcmp(path, drive_prof_csv)) {
            snprintf(drive_prof_csv, sizeof drive_prof_csv, "%s", path);
            drive_prof_csv_new = 1;
            if (drive_prof < 0 && *path && strcmp(path, "off")) drive_prof = 1;     /* (a CSV implies on) */
        }
    }
    if (strstr(cmd, "drive_prof") || strstr(cmd, "prof_every=")) drive_prof_apply();
    /* (engine only): threads per frame, their uclamp.min, CPU wakeup latency hold */
    if (strstr(cmd, "nwork=") || strstr(cmd, "rt_umin=") || strstr(cmd, "qos="))
        fprintf(stderr, "glassine-epdd-desktop: nwork=%d rt_umin=%d qos=%d (-1 global, -2 off, N us on the stream CPUs; "
                "qos: from the next link-up)\n", epd_stream_nwork, epd_stream_rt_umin, epd_stream_qos);
    if (strchr(cmd, '='))
        fprintf(stderr, "glassine-epdd-desktop: burst=%s du_max=%.3f flash_min=%.3f clean=%s ink=%s cut=%s%d dither=%s idle_ms=%d run_wait=%.2g/%d pace=%s motion=%s run_ms=%d run_fold=%d run_end=%s quiet_ms=%d quiet_after=%d src=%s\n",
                pol.burst_auto ? "auto" : "du", pol.du_max, pol.flash_min, pol.clean_full ? "full" : "partial",
                ink_mode == 0 ? "off" : "apps",
                pol.cut_auto ? "auto/" : "", pol.cut_auto ? pol.cut_cur : pol.cut_fixed, pol.dither_runs ? "on" : "off",
                pol.idle_ms, pol.run_wait, pol.run_wait_max, pol.pace ? "engine" : "fixed", pol.adaptive ? "adaptive" : "binary", pol.run_ms, pol.run_fold, pol.run_end == RUN_END_FLASH ? "flash" : "partial",
                pol.quiet_ms, pol.quiet_after, src_fb ? "fb" : "x");
}

static int64_t now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

/* portrait (X) rect -> panel rect. Portrait pixel (px,py) lands at panel
 * (x = PW-1-py, y = px) — the mapping glassine-epd-show uses for "ccw". */
#define TILE 32
#define TW ((PH + TILE - 1) / TILE)
#define TH ((PW + TILE - 1) / TILE)
static box tilebox[TW * TH];
/* battery temperature (the MAX17135 on the Note Air 1 has no thermistor)
 * in C, or 24 */
static int battery_celsius(void)
{
    static const char *p[] = { "/sys/class/power_supply/qcom-battery/temp", "/sys/class/power_supply/battery/temp" };
    for (unsigned i = 0; i < sizeof p / sizeof *p; i++) {
        FILE *f = fopen(p[i], "r");
        char b[32];
        if (!f) continue;
        int ok = fgets(b, sizeof b, f) != NULL;
        fclose(f);
        if (ok) return atoi(b) / 10;
    }
    return 24;
}

/* where an update's time goes before the engine (the "stats" command): the
 * read (compositor or X), scan + commit into the shadow, the hand-off */
static struct { unsigned long n, sub; double snap, snap_max, commit, submit; } tm;

static double tsec(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec / 1e9;
}


/* pace=engine: ms until a run's next read should start — the engine frees the
 * pending damage's area (epd_stream_free_ms), the read + hand-off take lead_ms
 * (measured on the run's bursts), plus a frame of margin. -1: no engine, or
 * not in a run (the policy then keeps its fixed gaps) */
static double lead_ms = 40;
static int eng_wait_box(box b);
/* in a run: ms until the engine could start a burst of pd, or of the last read
 * when nothing is pending — the quiet while ahead_due holds sway's frame is ours,
 * and the policy must see it (an OpenMW A/B: with -1 there the
 * still-anim hold never fired, each run ended ~0.3-1.2 s after its first DU) */
static int eng_wait(const boxlist *pd)
{
    if (!eng || !pol.pace || pol.state != S_RUN) return -1;
    if (!pd->n) return eng_wait_box(last_rd);
    box b = pd->b[0];
    for (int i = 1; i < pd->n; i++) b = box_union(b, pd->b[i]);
    return eng_wait_box(b);
}
static int eng_wait_box(box b)
{
    if (!eng || box_empty(b)) return -1;
    /* screen (portrait) -> panel rect, as send does */
    epd_rect r = { PW - b.y1, b.x0, b.y1 - b.y0, b.x1 - b.x0 };
    int w = epd_stream_free_ms(eng, r) - (int)(lead_ms + 12);
    return w > 0 ? w : 0;
}

/* stats: a pixel with more net drive frames than this (either polarity) counts
 * as DC-loaded — a full A2 one way is 9, so 60 = several unbalanced pushes */
#define DC_WARN 60
static int send_a2len;          /* send: the A2 cut of the update being sent (0 = none) */
static epd_wl *wl;                 /* (defined with the wl source below) */
/* ahead=predict: ms until sway should be answered (<= 0: now) — the next read
 * minus how long sway likely takes to deliver after DONE */
static int ahead_due(const boxlist *pd)
{
    if (wl_ahead != 2) return 0;
    int r = pd->n ? eng_wait(pd) : eng_wait_box(last_rd);   /* (also outside runs) */
    if (r < 0) return 0;                    /* no engine view: as run */
    return r - (int)epd_wl_chain_pct(wl, 75, 80);
}

/* ---- stage 5: update targets drawn on the GPU (gtarget=) ----
 * The ingest keeps a mirror of `shadow` on the GPU: a commit of grey the
 * ingest made goes through epd_ingest_mirror_commit (the same per-pixel
 * functions, on the GPU), every other shadow write (X frames, the pen, the
 * sleep banner, frames the CPU converted) is marked here and uploaded before
 * the next commit or target. The engine then gets a handle (a layer drawn
 * from the mirror) instead of the CPU's copy. The ingest's GL context is the
 * main thread's: targets are taken there only (the pen thread's updates stay
 * CPU copies). gt_dirty, gt_live: buf_mu */
static epd_ingest *ing;             /* (the dma-buf ingest, below) */
static int gt_mode;                 /* 0 off, 1 on, 2 check */
static int gt_live, gt_tried;       /* the mirror is up and given to the engine / set up once */
static int gt_refused;              /* consecutive refusals by the engine (its gpe off) */
static pthread_t main_th;
#define GT_DIRTY 64
static epd_ingest_rect gt_dirty[GT_DIRTY];
static int gt_ndirty;
static struct { unsigned long subs, cpu, chk, bad, gcommits, ucommits, ups; unsigned long long uppx; } tmt;

static long rect_area(epd_ingest_rect r) { return (long)r.w * r.h; }

/* a shadow write the mirror does not have (panel rect: x = column, y = row) */
static void gt_mark(epd_ingest_rect r)
{
    if (!gt_live || r.w <= 0 || r.h <= 0) return;
    if (gt_ndirty) {                /* a pen stroke: pieces next to each other, one rect */
        epd_ingest_rect *l = &gt_dirty[gt_ndirty - 1];
        int x0 = l->x < r.x ? l->x : r.x, y0 = l->y < r.y ? l->y : r.y;
        int x1 = l->x + l->w > r.x + r.w ? l->x + l->w : r.x + r.w, y1 = l->y + l->h > r.y + r.h ? l->y + l->h : r.y + r.h;
        epd_ingest_rect u = { x0, y0, x1 - x0, y1 - y0 };
        if (rect_area(u) <= rect_area(*l) + rect_area(r) + 4096) { *l = u; return; }
    }
    if (gt_ndirty == GT_DIRTY) { gt_ndirty = 1; gt_dirty[0] = (epd_ingest_rect){ 0, 0, PW, PH }; return; }
    gt_dirty[gt_ndirty++] = r;
}

static void gt_flush(void)
{
    for (int i = 0; i < gt_ndirty; i++) {
        epd_ingest_mirror_upload(ing, gt_dirty[i], shadow);
        tmt.ups++; tmt.uppx += (unsigned long long)rect_area(gt_dirty[i]);
    }
    gt_ndirty = 0;
}

static void gt_off(const char *why)
{
    if (!gt_live) return;
    gt_live = 0;
    gt_ndirty = 0;
    epd_stream_set_gt(eng, NULL);   /* (pending GPU targets become CPU copies) */
    fprintf(stderr, "glassine-epdd-desktop: GPU update targets OFF (%s)\n", why);
}

/* once: the ingest open (sway's first dma-buf), the engine with its state on
 * the GPU; main thread, buf_mu */
static void gt_setup(void)
{
    gt_tried = 1;
    if (!eng || epd_stream_gpu != 1 || !epd_stream_gpe || epd_stream_gpu_dead(eng)) {
        fprintf(stderr, "glassine-epdd-desktop: no GPU update targets (they need gpu=on gpe=on)\n");
        return;
    }
    if (epd_ingest_mirror(ing, shadow, EPD_GT_MAX)) return;
    gt_live = 1;
    gt_ndirty = 0;
    gt_refused = 0;
    epd_stream_set_gt(eng, epd_ingest_provider(ing));
    fprintf(stderr, "glassine-epdd-desktop: GPU update targets on%s\n", gt_mode == 2 ? ", checked against the CPU's" : "");
}

static int gt_main(void) { return pthread_equal(pthread_self(), main_th); }

/* gtarget=check: the layer read back == the CPU's target */
static int gt_check(int h, epd_rect r)
{
    epd_rect c = r;
    uint8_t *t = epd_target_make(&c, shadow, PW);
    if (!t) return 1;               /* (empty: nothing to compare) */
    uint8_t *b = malloc((size_t)c.w * c.h);
    long bad = -1;
    if (b && epd_ingest_target_read(ing, h, (epd_ingest_rect){ c.x, c.y, c.w, c.h }, b, 500) == 0) {
        bad = 0;
        for (long i = 0; i < (long)c.w * c.h; i++) bad += b[i] != t[i];
    }
    if (bad) fprintf(stderr, "glassine-epdd-desktop: GPU target %d,%d %dx%d: %s\n", c.x, c.y, c.w, c.h,
                     bad < 0 ? "no readback" : "differs from the CPU's");
    if (bad > 0) fprintf(stderr, "glassine-epdd-desktop: (%ld px)\n", bad);
    epd_target_free(t); free(b);
    if (bad) tmt.bad++; else tmt.chk++;
    return bad == 0;
}

/* the hand-off with a GPU target: 1 = done, 0 = the caller submits the CPU copy */
static int gt_submit(epd_rect r, int mode, int full)
{
    if (!gt_live || !gt_main() || epd_ingest_dead(ing)) return 0;
    gt_flush();
    int h = epd_ingest_target(ing, (epd_ingest_rect){ r.x, r.y, r.w, r.h });
    if (h <= 0) return 0;
    const epd_gt_provider *pv = epd_ingest_provider(ing);
    if (gt_mode == 2 && !gt_check(h, r)) { pv->release(pv->ctx, h); gt_off("a target differed"); return 0; }
    if (!epd_stream_submit_gt(eng, r, mode, full, h, shadow, PW)) {
        pv->release(pv->ctx, h);
        if (++gt_refused >= 16) gt_off("the engine refuses them: its state is off the GPU");
        return 0;
    }
    gt_refused = 0;
    tmt.subs++;
    return 1;
}

static void eng_update(const struct onyx_epdc_update *u)
{
    double t0 = tsec();
    if (now_ms() - eng_temp_ms > 60000) {           /* waveforms change per ~3 C */
        eng_temp_ms = now_ms();
        epd_stream_set_temp(eng, temp_override > -100 ? temp_override : battery_celsius());
    }
    int mode = u->waveform_mode == WF_DU4 ? EPD_DU4 : u->waveform_mode != WF_DU ? EPD_GC16 :
               eng_a2 || send_a2len ? EPD_MODE_A2LEN(EPD_A2_OR_DU, send_a2len) : EPD_DU;
    /* update_mode 2 (engine only): a flash of just the pixels that need one (epd_submit full 2) */
    epd_rect r = { u->x, u->y, u->width, u->height };
    int full = u->update_mode == 2 ? 2 : u->update_mode == 1;
    if (!gt_submit(r, mode, full)) {
        if (gt_live) tmt.cpu++;
        epd_stream_submit(eng, r, mode, full, shadow, PW);
    }
    tm.submit += tsec() - t0;
    tm.sub++;
    if (MP_ON && pthread_equal(pthread_self(), main_thread)) {     /* (the pen thread's submits: not the burst's) */
        double mk, lk, tg;
        epd_stream_last_submit(eng, &mk, &lk, &tg);
        mcur[MF_TARGET] += (float)mk; mcur[MF_APPEND] += (float)lk; mcur[MF_SUBS] += 1;
    }
}

static void ebc_update(const struct onyx_epdc_update *u0, const char *what)
{
    struct onyx_epdc_update u = *u0;
    u.marker = ++marker;
    if (eng) {
        eng_update(&u);
        /* a2 = the cut (0: uncut); busy = how long the area is taken from now
         * on (the engine's epd_stream_free_ms, payback runs excluded) */
        /* hand-off = copy (the target, outside the lock) + lock (the wait for
         * the stream thread) + engine (epd_submit_target): */
        if (verbose) {
            double mk, lk, tg;
            epd_stream_last_submit(eng, &mk, &lk, &tg);
            fprintf(stderr, "update %s %s panel x=%d y=%d w=%d h=%d t=%lld a2 %d busy %d ms, hand-off copy %.1f lock %.1f engine %.1f ms\n", u.waveform_mode == WF_DU ? "DU  " : "GC16",
                    what, u.x, u.y, u.width, u.height, (long long)(now_ms() % 1000000), send_a2len,
                    epd_stream_free_ms(eng, (epd_rect){ u.x, u.y, u.width, u.height }), mk, lk, tg);
        }
        return;
    }
    if (ioctl(ebc, EBC_SEND_UPDATE, &u) < 0)
        perror("glassine-epdd-desktop: SEND_UPDATE");
    else if (verbose)
        fprintf(stderr, "update %s %s panel x=%d y=%d w=%d h=%d t=%lld\n", u.waveform_mode == WF_DU ? "DU  " : "GC16",
                what, u.x, u.y, u.width, u.height, (long long)(now_ms() % 1000000));
}

/* M_AREA — a flash of just this area. The driver widens EVERY update_mode=1
 * update to the whole panel (its region log: a 400x400 flash request ->
 * l=0 t=0 w=1872 h=1404): that was the "whole screen flashes while scrolling
 * Firefox". So an area flash is two partial GC16s of the area: the inverted
 * content, then the content. Every level n -> 15-n differs, so both drive every
 * pixel of the area (what clears ghosting) and nothing outside it. The driver
 * copies the buffer at SEND_UPDATE (glassine-flashtest inv/same), so it is restored
 * right away; the second update queues behind the first (collision list).
 * Cost (driver frame log): a GC16 is 38 frames at 11.3 ms whatever the
 * size, partial or full (~0.5 s); every SEND_UPDATE also burns ~47 ms of kernel
 * CPU even for a 16x16 no-op (measured). Full flash ~660 ms; local flash =
 * two in sequence, ~1 s. So an area >= flash_full of the screen (full-screen
 * page turns) takes the real full-panel flash: it would cover nearly everything
 * anyway, and is faster. */
static void send_u(box b, int wf, int mode);
/* n: an A2 cut (WF_DU on the engine, motion=adaptive) */
static void send(box b, int wf, int mode, int n)
{
    send_a2len = eng ? n : 0;
    if (!eng && wf == WF_DU4) wf = WF_DU;       /* (adaptive is engine only) */
    send_u(b, wf, mode);
    send_a2len = 0;
}
static void send_u(box b, int wf, int mode)
{
    struct onyx_epdc_update u = {0};
    if (mode == M_SCREEN) {
        u.x = 0; u.y = 0; u.width = PW; u.height = PH;
    } else {
        u.x = PW - b.y1; u.y = b.x0; u.width = b.y1 - b.y0; u.height = b.x1 - b.x0;
    }
    u.waveform_mode = wf;
    u.update_mode = mode == M_SCREEN ? 1 : 0;
    u.dither = 0x1000;
    u.flags = 0x20000;
    /* (the engine: no widening — flashing just the area costs nothing extra there,
     * and the widening flashed the static bar at every scroll end: -2 DC frames
     * on its black each time) */
    if (!eng && mode == M_AREA && (double)u.width * u.height >= flash_full * PW * PH) {
        u.x = 0; u.y = 0; u.width = PW; u.height = PH;
        u.update_mode = 1;
        ebc_update(&u, "FULL ");
        return;
    }
    if (mode != M_AREA) { ebc_update(&u, mode == M_SCREEN ? "FULL " : "part "); return; }
    if (eng) {          /* the engine flashes just the area, and there only what needs it */
        u.update_mode = 2;
        ebc_update(&u, "flash");
        return;
    }
    for (int y = u.y; y < u.y + u.height; y++)
        for (int x = u.x; x < u.x + u.width; x++) {
            uint8_t g = 255 - shadow[(size_t)y * PW + x];
            ebuf[(size_t)y * PW + x] = 0xFF000000u | g << 16 | g << 8 | g;
        }
    ebc_update(&u, "flash");
    for (int y = u.y; y < u.y + u.height; y++)
        for (int x = u.x; x < u.x + u.width; x++) {
            uint8_t g = shadow[(size_t)y * PW + x];
            ebuf[(size_t)y * PW + x] = 0xFF000000u | g << 16 | g << 8 | g;
        }
    ebc_update(&u, "flash");
}

/* Input boost: a touch, pen or key raises every cpufreq policy's minimum to its
 * top frequency until boost_ms after the last input. schedutil starts from the
 * idle clock (300 MHz) and ramps over tens of ms (rate_limit 20 ms per step),
 * while the first frames after input matter most: the app's repaint, X, the
 * compositor's read, the engine. Nothing else sets scaling_min_freq on the Note Air 1
 * so "off" writes cpuinfo_min_freq back; also done at start and exit,
 * so a crash while boosted is undone by the supervisor's restart. */
static void vlog(const char *fmt, ...);
static int boosted;
static int64_t boost_until;
static unsigned long boosts;

static void boost_set(int on)
{
    if (!eng) return;                   /* the stock kernel: its own cpufreq setup */
    for (int i = 0; i < 8; i++) {
        char p[96], v[32] = "";
        snprintf(p, sizeof p, "/sys/devices/system/cpu/cpufreq/policy%d/cpuinfo_%s_freq", i, on ? "max" : "min");
        FILE *f = fopen(p, "r");
        if (!f) continue;
        int ok = fgets(v, sizeof v, f) != NULL;
        fclose(f);
        snprintf(p, sizeof p, "/sys/devices/system/cpu/cpufreq/policy%d/scaling_min_freq", i);
        if (ok && (f = fopen(p, "w"))) { fputs(v, f); fclose(f); }
    }
    boosted = on;
    if (on) boosts++;
}

static void boost(void)
{
    static int64_t last;
    if (now_ms() - last > 1000) vlog("input after idle (boost %s)\n", boost_ms > 0 ? "on" : "off");  /* latency reading */
    last = now_ms();
    if (boost_ms <= 0) return;
    boost_until = now_ms() + boost_ms;
    if (!boosted) {
        double t0 = tsec();
        boost_set(1);
        vlog("boost on: %.2f ms\n", (tsec() - t0) * 1e3);
    }
}

/* the ms until the boost ends (-1: none); ends it when due. boost_run=on holds it
 * while the policy is in a motion run (a fling coasts on after the finger lifts,
 * and schedutil drops to 300 MHz within it), up to 3 s after the last input. The
 * run's own steps wake the loop, so the hold returns the time to that cap, never
 * a short poll: a 20 ms one changed the run's pacing (gaps 90 -> 74 ms) and
 * brought 5x the GC16s) */
static int boost_tick(void)
{
    if (!boosted) return -1;
    int64_t dt = boost_until - now_ms();
    if (dt > 0 && boost_ms > 0) return (int)dt;
    if (boost_run && boost_ms > 0 && pol.state == S_RUN && dt > -3000) return (int)(3000 + dt);
    boost_set(0);
    return -1;
}

/* any pen/touch/X input: the policy's clock, the CPU boost, and (engine) keep
 * the link and rails up so the update it causes starts without the ~75 ms
 * wake-up */
static void on_input(void)
{
    pol_input(&pol, now_ms());
    boost();
    if (eng && eng_keepalive_ms > 0) epd_stream_keepalive(eng, eng_keepalive_ms);
}

/* the driver's DU frame cut (the policy only asks with no DU in flight) */
static void set_cut(int n)
{
    if (eng) {
        epd_stream_set_cut(eng, n);
        if (verbose) fprintf(stderr, "cut %d\n", n);
        return;
    }
    int fd = open("/sys/devices/sepdc/cut_frame_num", O_WRONLY);
    if (fd < 0) { pol.cut_cur = -1; return; }
    char b[8];
    int k = snprintf(b, sizeof b, "%d\n", n);
    if (write(fd, b, k) == k) {
        if (verbose) fprintf(stderr, "cut_frame_num %d\n", n);
    } else pol.cut_cur = -1;            /* not set: ask again next time */
    close(fd);
}


/* The controller keeps its own idea of every pixel's level (the "old" half of
 * the waveform index) and a partial update never drives a pixel whose old and
 * new level match. DU only drives pixels towards black/white: a grey target
 * is not driven, yet the controller records it as reached — and the later GC16
 * partial cleanup then skips it (grey text changes that never showed up, window
 * remains after a switch, menus painted half-way; ).
 * So `shadow` == ebuf == the controller's belief, always, and a pixel enters the
 * ebuf only right before an update that can actually take it there:
 * C_ALL: everything (GC16 follows); C_BW: only black/white targets (DU follows),
 * greys keep their old value and stay "changed" until the idle GC16 cleanup. */
/* C_DITHER: greys become black/white by an ordered threshold in panel
 * coordinates (dith, below) — what DU actually shows, and what the controller
 * then believes, so the later GC16 drives them to the real grey. C_THRESH: the
 * same with one threshold (mid grey): clean black/white areas instead of dots */
static int bw_level(uint8_t g) { return (g >> 4) == 0 || (g >> 4) == 15; }

/* ---- where the pixels come from ---------------------------------------------
 * A frame = one portrait rect's pixels as panel-order grey levels (gamma
 * applied): row by = portrait px, column bx = PW-1-py, so every pass over it
 * (compare, commit into shadow + ebuf) runs sequentially in panel memory.
 *
 * Source x (default): XShmGetImage where the kernel has SysV shared memory
 * (mainline: Xorg copies the rect into our segment, nothing goes through the
 * socket), else XGetImage (the stock 4.4 kernel has no SysV IPC).
 * Source fb (src=fb): Xorg's fbdev driver renders into a shadow buffer and its
 * "Rotate CCW" copies every change into /dev/fb0 — already landscape (turned
 * 180 degrees against the panel buffer: a row just reads backwards). Reading it replaces XGetImage, which pushed the
 * whole rect through the X socket (no MIT-SHM here: no SysV IPC) and made Xorg
 * work too — but fb0 is uncached like /dev/ebc (~165 MB/s): measured per full
 * screen fb0 = 64 ms CPU, XGetImage = ~40 ms glassine-epdd-desktop + 12 ms Xorg, and
 * over a 10 s repaint test fb0 was no cheaper in total. So x stays the default;
 * each fb0 row is read once, in one go, and converted from a cached copy.
 * The shadow copy to fb0 happens in Xorg's block handler, before it flushes
 * replies: an XSync before reading = fb0 holds everything drawn so far.
 * "srccheck" compares both pixel by pixel (0 of 2.6 M differ, once the
 * software cursor is hidden and X's channel layout is used). */
/* g: row r (portrait column b.x0 + r) at g + r * s; s = w for a frame of its
 * own, PW for one that is a view of wlsrc (sway: snap_wl, no copy) */
typedef struct { box b; int bx0, w, h; uint8_t *g; int s; } frame;
static uint32_t *fbmem;
static int fb_stride;
/* channel positions from X's visual, for both sources: X writes the pixels in
 * its visual's layout (fb0's own red/blue offsets disagree: +-1 grey level on
 * tinted panel greys, srccheck) */
static int fb_rsh = 16, fb_gsh = 8, fb_bsh = 0;
static uint32_t *rowbuf;

static int fb_open(int fd)
{
    struct fb_var_screeninfo v;
    struct fb_fix_screeninfo f;
    if (fd < 0 || ioctl(fd, FBIOGET_VSCREENINFO, &v) < 0 || ioctl(fd, FBIOGET_FSCREENINFO, &f) < 0) return -1;
    if (v.xres != PW || v.yres < PH || v.bits_per_pixel != 32 || f.line_length < PW * 4) {
        fprintf(stderr, "glassine-epdd-desktop: /dev/fb0 is %ux%u/%u bpp — reading X instead\n", v.xres, v.yres, v.bits_per_pixel);
        return -1;
    }
    void *m = mmap(NULL, (size_t)f.line_length * PH, PROT_READ, MAP_SHARED, fd, 0);
    if (m == MAP_FAILED) return -1;
    fbmem = m;
    fb_stride = f.line_length / 4;
    rowbuf = malloc(PW * 4);
    return 0;
}

static inline uint8_t grey_of(uint32_t p, int rs, int gs, int bs)
{
    return lut[(((p >> rs) & 0xff) * 77 + ((p >> gs) & 0xff) * 150 + ((p >> bs) & 0xff) * 29) >> 8];
}

/* MIT-SHM: one full-screen segment, one rect at a time (snap runs on the
 * main thread only) */
static XShmSegmentInfo shm = { .shmid = -1 };
static int shm_ok, shm_err;
static Visual *shm_vis;
static int shm_depth;

static int shm_xerr(Display *d, XErrorEvent *e) { (void)d; (void)e; shm_err = 1; return 0; }

static void shm_init(Display *dpy, int scr, int W, int H)
{
    if (!XShmQueryExtension(dpy)) return;
    shm.shmid = shmget(IPC_PRIVATE, (size_t)W * H * 4, IPC_CREAT | 0600);
    if (shm.shmid < 0) {
        if (verbose) fprintf(stderr, "glassine-epdd-desktop: no SysV shm (%s): XGetImage\n", strerror(errno));
        return;
    }
    shm.shmaddr = shmat(shm.shmid, NULL, 0);
    shm.readOnly = False;
    if (shm.shmaddr == (char *)-1) { shmctl(shm.shmid, IPC_RMID, NULL); return; }
    XErrorHandler old = XSetErrorHandler(shm_xerr);
    shm_err = 0;
    Status st = XShmAttach(dpy, &shm);
    XSync(dpy, False);
    XSetErrorHandler(old);
    shmctl(shm.shmid, IPC_RMID, NULL);          /* goes once X and we detach */
    if (!st || shm_err) { shmdt(shm.shmaddr); fprintf(stderr, "glassine-epdd-desktop: XShmAttach failed: XGetImage\n"); return; }
    shm_vis = DefaultVisual(dpy, scr);
    shm_depth = DefaultDepth(dpy, scr);
    shm_ok = 1;
    fprintf(stderr, "glassine-epdd-desktop: grabs via MIT-SHM\n");
}

static XImage *get_image(Display *dpy, Window root, box b)
{
    int w = b.x1 - b.x0, h = b.y1 - b.y0;
    if (shm_ok) {
        XImage *img = XShmCreateImage(dpy, shm_vis, shm_depth, ZPixmap, shm.shmaddr, &shm, w, h);
        if (img && img->bits_per_pixel == 32 && XShmGetImage(dpy, root, img, b.x0, b.y0, AllPlanes)) return img;
        if (img) { img->data = NULL; XDestroyImage(img); }
        shm_ok = 0;                               /* once: don't fail on every grab */
        fprintf(stderr, "glassine-epdd-desktop: XShmGetImage failed: XGetImage from now on\n");
    }
    return XGetImage(dpy, root, b.x0, b.y0, w, h, AllPlanes, ZPixmap);
}

static void put_image(XImage *img)
{
    if (img->data == shm.shmaddr) img->data = NULL;   /* the segment isn't ours to free */
    XDestroyImage(img);
}

static int snap_x(Display *dpy, Window root, frame *f)
{
    box b = f->b;
    XImage *img = get_image(dpy, root, b);
    if (!img) return -1;
    for (int py = b.y0; py < b.y1; py++) {     /* (bands of rows to write the frame
                                                   * sequentially: slower) */
        const uint32_t *row = (const uint32_t *)((const uint8_t *)img->data + (size_t)(py - b.y0) * img->bytes_per_line);
        for (int px = b.x0; px < b.x1; px++)
            f->g[(size_t)(px - b.x0) * f->w + (b.y1 - 1 - py)] = grey_of(row[px - b.x0], fb_rsh, fb_gsh, fb_bsh);
    }
    put_image(img);
    return 0;
}

/* fb0 is the panel buffer turned 180 degrees (Xorg's CCW vs the panel's
 * mapping; , checked with srccheck): portrait (px,py) sits at fb0
 * (py, PH-1-px), panel (PW-1-py, px). A panel row is an fb0 row backwards. */
static int snap_fb(frame *f)
{
    for (int by = f->b.x0; by < f->b.x1; by++) {
        memcpy(rowbuf, fbmem + (size_t)(PH - 1 - by) * fb_stride + (PW - f->bx0 - f->w),
               (size_t)f->w * 4);                                   /* one uncached read */
        uint8_t *g = f->g + (size_t)(by - f->b.x0) * f->w;
        for (int i = 0; i < f->w; i++) g[i] = grey_of(rowbuf[f->w - 1 - i], fb_rsh, fb_gsh, fb_bsh);
    }
    return 0;
}

/* Frame buffers, reused: a read's 2.6 MB malloc'd and freed each time went
 * back to the kernel (musl unmaps large blocks), so every read page-faulted
 * ~640 fresh zeroed pages, billed to whoever wrote the frame first (the
 * compositor's rotation measured 8.5 ns/px in here, 2 in a benchmark). A few
 * slots, grown on demand; main thread only (snap's callers). */
#define NFBUF 8
static struct { uint8_t *p; size_t cap; int used; } fbuf[NFBUF];

static uint8_t *fbuf_get(size_t n)
{
    int k = -1;
    for (int i = 0; i < NFBUF; i++)
        if (!fbuf[i].used && (k < 0 || fbuf[i].cap > fbuf[k].cap)) k = i;
    if (k < 0) return malloc(n);                /* all in use: a plain one */
    if (fbuf[k].cap < n) {
        uint8_t *p = realloc(fbuf[k].p, n);
        if (!p) return NULL;
        fbuf[k].p = p;
        fbuf[k].cap = n;
    }
    fbuf[k].used = 1;
    return fbuf[k].p;
}

/* sway's frame, panel layout: wlsrc (ours, rows PW apart), or (ingest_view,
 * ) the backend's R8 buffer read in place (wl_vh: its ingest
 * handle, rows wl_st apart, mapped read-only). The readers take wl_base /
 * wl_st (snap_wl); anything that writes the frame calls wl_own first */
static uint8_t *wlsrc;
static const uint8_t *wl_base;
static int wl_st = PW, wl_vh = -1;
static struct { unsigned long n, own; double sync, own_s; } tmv;   /* (stats: frames read in place, copies back) */

static void fbuf_put(uint8_t *p)
{
    if (!p || (wlsrc && p >= wlsrc && p < wlsrc + (size_t)PW * PH)) return;     /* (a view of wlsrc: snap_wl) */
    if (ing && epd_ingest_owns(ing, p)) return;                                 /* (a view of the buffer read in place) */
    for (int i = 0; i < NFBUF; i++)
        if (fbuf[i].p == p) { fbuf[i].used = 0; return; }
    free(p);
}

/* portrait rect b -> frame (NULL g on failure; give g back with fbuf_put) */
/* a tile handle being dragged (see drag_cmd): its pixels, where they are
 * shown now, the X window reads leave out */
static struct {
    int on;
    Window win;
    box cur, orig;              /* orig: the handle's X place, the tiles' gap */
    frame bar;
    int64_t last;
} dg;

/* every read: the dragged bar where it is shown, not what X has there; the
 * gap it left (no window there: X shows the wallpaper) plain white */
static void drag_stamp(frame *f)
{
    if (!dg.on) return;
    box o = box_clip(f->b, dg.orig);
    for (int px = o.x0; !box_empty(o) && px < o.x1; px++)
        memset(f->g + (size_t)(px - f->b.x0) * f->w + (f->b.y1 - o.y1), lut[255], o.y1 - o.y0);
    o = box_clip(f->b, dg.cur);
    if (box_empty(o)) return;
    for (int px = o.x0; px < o.x1; px++) {
        uint8_t *d = f->g + (size_t)(px - f->b.x0) * f->w;
        const uint8_t *s = dg.bar.g + (size_t)(px - dg.cur.x0) * dg.bar.w;
        for (int py = o.y0; py < o.y1; py++) d[f->b.y1 - 1 - py] = s[dg.cur.y1 - 1 - py];
    }
}

/* GLASSINE_EPDD_SRC=wl (: the frames come
 * from sway's glassine-epd output backend (epd-wl.c), not from X. Each frame's
 * damaged pixels are turned into grey at once, into wlsrc (panel layout, like
 * shadow), and the frame is answered (DONE) once its damage has been read:
 * sway draws the next one only then, so apps draw at the panel's pace. Reads
 * (snap) copy from wlsrc. No X: no window events, profiles by window, drags,
 * impostors, parking — dpy is NULL. */
static epd_wl *wl;
static boxlist wl_dmg;              /* what the last frames changed (portrait), for the main loop */
/* the convert (XRGB -> grey, rotated into wlsrc); "stats" prints and resets */
static struct { unsigned long n; unsigned long long px; double s, max; } tmw;

static void grey_rect(uint8_t *dst, const uint32_t *px, int stride, int x, int y, int w, int h)
{
    epd_grey_rect_mode(grey_lum ? lut_lum : lut, grey_lum, dst, PW, px, stride, x, y, w, h);
}

/* sway's grey output carries q by the formula its HELLO announced */
static const uint8_t *r8_lut_real(void) { return epd_wl_caps(wl) & GLASSINE_EPD_CAP_LUM ? lut_lum : lut; }
/* ... unless that frame's buffer went through it already (sway's final pass,
 * GLASSINE_EPD_CAP_GAMMA): then the identity, which the ingest copies */
static const uint8_t *r8_lut(void)
{
    static uint8_t id[256];
    if (!id[255]) for (int i = 0; i < 256; i++) id[i] = (uint8_t)i;
    return epd_wl_prelut(wl) ? id : r8_lut_real();
}

/* the mouse pointer (sway): sway's glassine-epd backend forwards its
 * cursor (epd-wl: the sprite and its box, portrait). Its sprite is made
 * black/white once (alpha < 128: not part of it; else luminance < 128 black,
 * white), through the frames' LUT, in a frame's layout, and STAMPED into every
 * read that crosses it (snap: a copy of the wlsrc view, so commit uploads
 * it to the GPU mirror instead of committing the ingest's grey): an app's
 * repaint under the pointer, the cleanup's grabs all contain it. Where it
 * moves, ptr_step paints it itself (pol_pointer: the fast end of the motion
 * range), the GC16 partial when it rests. Never while the pen inks, nor
 * PTR_PEN_MS after pen input (sway hides it for the pen: 0009); pointer=off:
 * never. Main thread only */
#define PTR_PEN_MS 300
#define PTR_IMG_MS 1000                 /* a sprite-only change (an animated cursor): one repaint per this */
#define PTR_MIN_MS 24                   /* paints at least this far apart (RUN_FLOOR_MS: two drive frames) */
static struct {
    int img;                            /* a sprite (shown when ptr_vis) */
    box b;                              /* its box, portrait, not clipped */
    int fw;                             /* b.y1 - b.y0: a row's length (frame layout) */
    uint8_t g[GLASSINE_EPD_CUR_MAX * GLASSINE_EPD_CUR_MAX], m[GLASSINE_EPD_CUR_MAX * GLASSINE_EPD_CUR_MAX];   /* grey, mask */
    unsigned gen;                       /* sprite changes */
} ptr;
static int ptr_vis;                     /* shown now (ptr_mode, a sprite, no pen): set once per loop */
static struct { int on; box b; unsigned gen; int64_t t, t_img; } ptr_drawn;   /* what the last pointer paint showed */
static unsigned long ptr_paints;

static void ptr_cb(void *ctx, const epd_wl_cursor *c)
{
    (void)ctx;
    if (c->moved && c->on) on_input();  /* a mouse is input: the quiet pacing, the boost, the link */
    if (!c->on) { ptr.img = 0; return; }
    ptr.b = (box){ c->r.x, c->r.y, c->r.x + c->r.w, c->r.y + c->r.h };
    ptr.fw = c->r.h;
    if (c->img || !ptr.img) {
        const uint8_t *t = c->panel ? r8_lut_real() : grey_lum ? lut_lum : lut;
        for (int py = 0; py < c->r.h; py++)
            for (int px = 0; px < c->r.w; px++) {
                uint32_t p = c->px[(size_t)py * c->r.w + px];
                unsigned a = p >> 24, l = (((p >> 16) & 0xff) * 77 + ((p >> 8) & 0xff) * 150 + (p & 0xff) * 29) >> 8;
                size_t i = (size_t)px * ptr.fw + (ptr.fw - 1 - py);     /* row = portrait x, i = b.y1 - 1 - y */
                ptr.m[i] = a >= 128;
                ptr.g[i] = l * 255 < 128 * a ? t[0] : t[255];       /* (premultiplied) */
            }
        ptr.gen++;
    }
    ptr.img = 1;
}

/* the pointer into a read of sway's frames: f's pixels become a copy (fbuf) */
static void ptr_stamp(frame *f)
{
    if (!ptr_vis) return;
    box o = box_clip(f->b, ptr.b);
    if (box_empty(o)) return;
    uint8_t *g = fbuf_get((size_t)f->w * f->h);
    if (!g) return;
    for (int r = 0; r < f->h; r++) memcpy(g + (size_t)r * f->w, f->g + (size_t)r * f->s, (size_t)f->w);
    f->g = g;
    f->s = f->w;
    for (int px = o.x0; px < o.x1; px++) {
        uint8_t *d = f->g + (size_t)(px - f->b.x0) * f->w;
        const uint8_t *sg = ptr.g + (size_t)(px - ptr.b.x0) * ptr.fw, *sm = ptr.m + (size_t)(px - ptr.b.x0) * ptr.fw;
        for (int py = o.y0; py < o.y1; py++)
            if (sm[ptr.b.y1 - 1 - py]) d[f->b.y1 - 1 - py] = sg[ptr.b.y1 - 1 - py];
    }
}

/* the frame back into wlsrc (the whole picture, from the buffer read in place)
 * and that buffer let go: before anything writes wlsrc. Its DONE is epd-wl's
 * (the convert this runs in, or epd_wl_unkeep) */
static void wl_own(void)
{
    if (wl_vh < 0) return;
    double t0 = tsec();
    for (int r = 0; r < PH; r++) memcpy(wlsrc + (size_t)r * PW, wl_base + (size_t)r * wl_st, PW);
    if (ing) epd_ingest_view(ing, wl_vh, 0, NULL, NULL, NULL);
    wl_vh = -1; wl_base = wlsrc; wl_st = PW;
    dz_prep_drop();
    tmv.own++; tmv.own_s += tsec() - t0;
}

/* ingest_view=off: no frame read in place from now on */
static void wl_view_off(void)
{
    if (wl_vh < 0) return;
    wl_own();
    if (wl) epd_wl_unkeep(wl);
}

/* a grey frame's panel rect (sway's grey output): only the LUT */
static void grey_lut_rect(uint8_t *dst, const uint8_t *g, int stride, int x, int y, int w, int h)
{
    const uint8_t *t = r8_lut();
    for (int r = 0; r < h; r++) epd_grey_lut(t, dst + (size_t)(y + r) * PW + x, g + (size_t)r * stride, w);
}

static void wl_grey(void *ctx, const uint8_t *g, int stride, int x, int y, int w, int h)
{
    (void)ctx;
    wl_own();
    double t0 = tsec();
    grey_lut_rect(wlsrc, g, stride, x, y, w, h);
    int px = y, py = PW - x - w, pw = h, ph = w;        /* the portrait rect */
    if (ing) epd_ingest_stale(ing, (epd_ingest_rect){ px, py, pw, ph });
    if (!wl_lazy) list_add(&wl_dmg, (box){ px, py, px + pw, py + ph });
    double dt = tsec() - t0;
    MADD(MF_CONV, dt * 1e3);
    mo_ing_t = t0 + dt;
    tmw.n++; tmw.px += (unsigned long long)w * h; tmw.s += dt;
    if (dt > tmw.max) tmw.max = dt;
}

static void wl_rect(void *ctx, const uint32_t *px, int stride, int x, int y, int w, int h)
{
    (void)ctx;
    wl_own();
    double t0 = tsec();
    grey_rect(wlsrc, px, stride, x, y, w, h);
    if (ing) epd_ingest_stale(ing, (epd_ingest_rect){ x, y, w, h });     /* (the mirror's commits: not the ingest's grey) */
    if (!wl_lazy) list_add(&wl_dmg, (box){ x, y, x + w, y + h });   /* lazy: wl_damage had it */
    double dt = tsec() - t0;
    MADD(MF_CONV, dt * 1e3);
    mo_ing_t = t0 + dt;
    tmw.n++; tmw.px += (unsigned long long)w * h; tmw.s += dt;
    if (dt > tmw.max) tmw.max = dt;
}

/* dma-buf frames (GLES sway) through epd-ingest. Opened at the
 * first dma-buf; one that fails to open (or a dead GPU) leaves the CPU way */
static epd_ingest *ing;
static int ing_failed;
static uint8_t *ing_chk;            /* ingest=check: the CPU's grey */
static struct { unsigned long n, chk, bad; unsigned long long px, badpx; double s, max; } tmg;
/* where a GPU ingest's time goes, per app (the "ingest" ms were
 * mostly the wait for the app's + sway's GPU work, not our shader):
 * wait poll(POLLIN) on the dma-buf until its write fences signalled (the
 * app's frame, then sway's composite into this buffer; ingwait=on)
 * gpu our draw calls until their fence (+ the ring's queue ahead of them)
 * read glReadPixels
 * the app: the one whose window has most of the frame's damage ("-": none) */
/* the visible windows (glassine-swayd's "app" hints, portrait), see hint_cmd */
#define MAXWL 32
static struct { box b; char cls[128]; } wlapp[MAXWL];
#define MAXSHELL 16
static box shellr[MAXSHELL];            /* the shell's mapped surfaces ("shell ID X Y W H" .. "shell gone ID") */
static unsigned shellid[MAXSHELL];
static int nshell;
static int nwlapp;
#define INGAPPS 8
static struct ingapp {
    char cls[48];
    unsigned long n, slow;              /* slow: frames > 20 ms in all */
    unsigned long long px;
    double wait, gpu, read, wmax, gmax, rmax;
} tmapp[INGAPPS];

static struct ingapp *ingapp_of(const epd_wl_rect *r, int n)
{
    long best = 0;
    const char *cls = "-";
    for (int i = 0; i < nwlapp; i++) {
        long a = 0;
        for (int k = 0; k < n; k++) {
            box d = box_clip((box){ r[k].x, r[k].y, r[k].x + r[k].w, r[k].y + r[k].h }, wlapp[i].b);
            if (!box_empty(d)) a += box_area(d);
        }
        if (a > best) { best = a; cls = wlapp[i].cls; }
    }
    struct ingapp *free_ = NULL;
    for (int i = 0; i < INGAPPS; i++) {
        if (!strncmp(tmapp[i].cls, cls, sizeof tmapp[i].cls - 1) && tmapp[i].cls[0]) return &tmapp[i];
        if (!tmapp[i].cls[0] && !free_) free_ = &tmapp[i];
    }
    if (!free_) free_ = &tmapp[INGAPPS - 1];   /* full: the last one takes the rest */
    if (!free_->cls[0]) snprintf(free_->cls, sizeof free_->cls, "%s", cls);
    return free_;
}

static void ingapp_stats(void)
{
    for (int i = 0; i < INGAPPS && tmapp[i].cls[0]; i++) {
        struct ingapp *a = &tmapp[i];
        double n = a->n ? a->n : 1;
        fprintf(stderr, "glassine-epdd-desktop: ingest %s: %lu frames (%lu > 20 ms), %.0f kpx avg; wait %.1f ms avg (max %.1f), "
                "gpu %.1f (max %.1f), read %.1f (max %.1f)\n", a->cls, a->n, a->slow, a->px / n / 1e3,
                a->wait * 1e3 / n, a->wmax * 1e3, a->gpu * 1e3 / n, a->gmax * 1e3, a->read * 1e3 / n, a->rmax * 1e3);
    }
    memset(tmapp, 0, sizeof tmapp);
}

static const uint8_t *ing_lut8;      /* the R8 LUT the ingest has */
static void ingest_lut(void)
{
    if (wl) epd_wl_set_luts(wl, lut, lut_lum);     /* (sway may apply them on its GPU) */
    if (!ing) return;
    epd_ingest_lut(ing, grey_lum ? lut_lum : lut);
    epd_ingest_mode(ing, grey_lum);
    epd_ingest_lut8(ing, ing_lut8 = r8_lut());
}

static int wl_dma_import(void *ctx, epd_wl_dmabuf *b)
{
    (void)ctx;
    if (!ing && !ing_failed) {
        if ((ing = epd_ingest_open(PW, PH, lut))) {
            fprintf(stderr, "glassine-epdd-desktop: ingest on %s\n", epd_ingest_renderer(ing));
            ingest_lut();
        } else ing_failed = 1;
    }
    if (!ing) return -1;
    b->handle = epd_ingest_import(ing, b->fd, b->w, b->h, b->stride, b->offset, b->format, b->modifier);
    return b->handle < 0 ? -1 : 0;
}

static void wl_dma_drop(void *ctx, epd_wl_dmabuf *b)
{
    (void)ctx;
    if (b->handle >= 0 && b->handle == wl_vh) wl_own();    /* (the buffer read in place goes: its picture stays) */
    if (ing) epd_ingest_drop(ing, b->handle);
}

static void chk_rect(void *ctx, const uint32_t *px, int stride, int x, int y, int w, int h)
{
    (void)ctx;
    grey_rect(ing_chk, px, stride, x, y, w, h);
}

static void chk_grey(void *ctx, const uint8_t *g, int stride, int x, int y, int w, int h)
{
    (void)ctx;
    grey_lut_rect(ing_chk, g, stride, x, y, w, h);
}

/* the GPU into wlsrc, or (ingest=cpu,) the ingest's
 * mapping of the buffer by the CPU: the same bytes, no job on the GPU's one
 * ring (its draw + fence were ing_gpu ~9 ms an update in front of the drive
 * passes); that works with a dead ingest context too. -1: epd-wl's CPU way
 * (no ingest, or it failed) */
static int wl_dma_frame(void *ctx, epd_wl_dmabuf *b, const epd_wl_rect *r, int n)
{
    (void)ctx;
    if (ing && ingest_mode && epd_ingest_dead(ing)) {   /* a slow GPU frame, not a hang: back after a while */
        static double next, gap = 2;
        double tn = tsec();
        if (tn >= next) {
            next = tn + gap;
            gap = gap < 30 ? gap * 2 : 60;
            if (epd_ingest_revive(ing)) fprintf(stderr, "glassine-epdd-desktop: ingest on the GPU again (its timed-out frame finished)\n");
        }
    }
    if (!ing || (ingest_mode && epd_ingest_dead(ing))) return -1;
    if (ing_lut8 != r8_lut()) ingest_lut();         /* (a new sway announced another formula) */
    double t0 = tsec();
    if (ingest_wait && b->fd >= 0) {    /* the producer's write fences (implicit sync) */
        struct pollfd pf = { b->fd, POLLIN, 0 };
        if (poll(&pf, 1, 500) == 0) vlog("ingest: the buffer's producer not done in 500 ms\n");
    }
    double tw = tsec() - t0;
    epd_ingest_times it;
    /* ingest_view : a frame that went through our table on sway's
     * GPU (its tag; the copy would be an identity) is read in place: the R8
     * buffer becomes the frame (wl_base), its DONE waits for the next one
     * (epd-wl's keep, answered NEXT now) */
    int kept = 0;
    if (!ingest_mode && ingest_view && epd_wl_prelut(wl) && b->format == 0x20203852u &&
        (epd_wl_caps(wl) & GLASSINE_EPD_CAP_NEXT)) {
        const uint8_t *m;
        int st;
        if (epd_ingest_view(ing, b->handle, 1, &m, &st, &it) == 0) {
            if (wl_vh >= 0 && wl_vh != b->handle) epd_ingest_view(ing, wl_vh, 0, NULL, NULL, NULL);
            wl_vh = b->handle; wl_base = m; wl_st = st;
            for (int k = 0; k < n; k++) epd_ingest_stale(ing, (epd_ingest_rect){ r[k].x, r[k].y, r[k].w, r[k].h });
            tmv.n++; tmv.sync += it.sync;
            kept = 1;
        }
    }
    if (!kept) {
        wl_own();
        if ((ingest_mode ? epd_ingest_frame_t(ing, b->handle, (const epd_ingest_rect *)r, n, wlsrc, 500, &it)
                         : epd_ingest_frame_cpu(ing, b->handle, (const epd_ingest_rect *)r, n, wlsrc, &it)) < 0) return -1;
    }
    double dt = tsec() - t0;
    tw += it.sync;                      /* (cpu: the START ioctl's wait is the producer's fence, as the poll) */
    if (MP_ON) {
        mcur[MF_ING_WAIT] += (float)(tw * 1e3); mcur[MF_ING_GPU] += (float)(it.gpu * 1e3);
        mcur[MF_ING_READ] += (float)(it.read * 1e3); mcur[MF_ING_N] += 1;
    }
    mo_fence_t = t0 + tw;
    mo_ing_t = t0 + dt;
    tmg.n++; tmg.s += dt;
    if (dt > tmg.max) tmg.max = dt;
    unsigned long long fpx = 0;
    for (int k = 0; k < n; k++) {
        fpx += (unsigned long long)r[k].w * r[k].h;
        if (!wl_lazy) list_add(&wl_dmg, (box){ r[k].x, r[k].y, r[k].x + r[k].w, r[k].y + r[k].h });
    }
    tmg.px += fpx;
    struct ingapp *a = ingapp_of(r, n);
    a->n++; a->px += fpx; a->wait += tw; a->gpu += it.gpu; a->read += it.read;
    if (tw > a->wmax) a->wmax = tw;
    if (it.gpu > a->gmax) a->gmax = it.gpu;
    if (it.read > a->rmax) a->rmax = it.read;
    if (dt > 0.020) a->slow++;
    vlog("ingest %s: wait %.1f gpu %.1f read %.1f ms, %d rects %llu kpx\n", a->cls, tw * 1e3, it.gpu * 1e3,
         it.read * 1e3, n, fpx / 1000);
    if (ingest_mode == 2 && (ing_chk || (ing_chk = malloc((size_t)PW * PH)))) {
        /* the CPU's grey of the same rects; where they differ the CPU's wins */
        if (epd_wl_cpu_rects(wl, b, r, n, chk_rect, chk_grey, NULL) == 0) {
            tmg.chk++;
            unsigned long long bad = 0;
            for (int k = 0; k < n; k++) {
                /* the GPU widened the columns to whole texels: compare what the CPU wrote */
                int c0 = PW - (r[k].y + r[k].h), c1 = PW - r[k].y;
                for (int row = r[k].x; row < r[k].x + r[k].w; row++) {
                    uint8_t *g = wlsrc + (size_t)row * PW, *cp = ing_chk + (size_t)row * PW;
                    for (int col = c0; col < c1; col++) if (g[col] != cp[col]) { bad++; g[col] = cp[col]; }
                }
            }
            if (bad) {
                for (int k = 0; k < n; k++) epd_ingest_stale(ing, (epd_ingest_rect){ r[k].x, r[k].y, r[k].w, r[k].h });
                if (tmg.bad < 5) fprintf(stderr, "glassine-epdd-desktop: ingest check: %llu px differ in a frame of %d rects\n", bad, n);
                tmg.bad++; tmg.badpx += bad;
            }
        }
    }
    return kept;
}

static const epd_wl_dmabuf_ops wl_dma_ops = { wl_dma_import, wl_dma_drop, wl_dma_frame };

/* lazy : a frame's damage at its arrival, its pixels only when a
 * read needs them (snap -> epd_wl_flush). In runs the early answer is NEXT:
 * sway draws the next frame while this one stays locked; a frame replaced
 * before a read is answered unread (~40 % of them in a scroll: the ingest's
 * GPU work + readback, or the CPU convert, saved) */
static void wl_damage(void *ctx, const epd_wl_rect *r, int n)
{
    (void)ctx;
    for (int k = 0; k < n; k++) list_add(&wl_dmg, (box){ r[k].x, r[k].y, r[k].x + r[k].w, r[k].y + r[k].h });
}

/* the dead band's true frames : their rects, until the main
 * loop has compared them with the panel (true_frames). Not damage */
static boxlist wl_true;
static unsigned long true_n, true_changed;

static void wl_true_rects(void *ctx, const epd_wl_rect *r, int n)
{
    (void)ctx;
    for (int k = 0; k < n; k++) list_add(&wl_true, (box){ r[k].x, r[k].y, r[k].x + r[k].w, r[k].y + r[k].h });
    true_n++;
}

/* quiet: half the settle, so the true picture is there when the cleanup reads */
static void band_push(void)
{
    if (wl) epd_wl_set_band(wl, band_t, pol.idle_ms / 2, wl_true_rects, NULL);
}

static void wl_lazy_set(int on)
{
    if (wl) {
        epd_wl_flush(wl);               /* nothing held across the switch */
        epd_wl_set_damage(wl, on ? wl_damage : NULL, NULL);
    }
    wl_lazy = on;
}

/* sway: the frame is a view of wlsrc (the per-read memcpy, 1 B/px,
 * went). wlsrc changes only in epd_wl_dispatch at the top of the main loop, so
 * nothing writes it between a read and its commit; fbuf_put ignores the view */
static void snap_wl(frame *f)
{
    epd_wl_flush(wl);                   /* the newest frame's pixels, if not yet (lazy) */
    f->g = (uint8_t *)wl_base + (size_t)f->b.x0 * wl_st + f->bx0;     /* (read-only when read in place) */
    f->s = wl_st;
}

/* 1: f is a view of sway's frame (snap_wl), wlsrc or the buffer read in place */
static int wl_is_view(const frame *f)
{
    return wl_base && f->s == wl_st && f->g == wl_base + (size_t)f->b.x0 * wl_st + f->bx0;
}

static frame snap(Display *dpy, Window root, box b, int use_fb)
{
    frame f = { b, PW - b.y1, b.y1 - b.y0, b.x1 - b.x0, NULL, b.y1 - b.y0 };
    if (box_empty(b)) return f;
    if (wl) { snap_wl(&f); ptr_stamp(&f); return f; }      /* (no drags under sway; the pointer) */
    if (!(f.g = fbuf_get((size_t)f.w * f.h))) { f.g = NULL; return f; }
    int r;
    if (cm) r = comp_read(cm, b, f.g);          /* the windows, composited (epd-comp.c) */
    else if (use_fb && fbmem) {
        XSync(dpy, False);              /* Xorg has copied everything drawn so far into fb0 */
        r = snap_fb(&f);
    } else r = snap_x(dpy, root, &f);
    if (r < 0) { fbuf_put(f.g); f.g = NULL; }
    else drag_stamp(&f);
    return f;
}

/* visibly changed (the panel's 16 levels, the high nibble): the first / last
 * i in [a, b] (inclusive) where sh and g differ, -1: none; 8 at a time, straight from the two rows
 * (changed_row's flag row + first_nz/last_nz read every byte of
 * every row; a scroll changes nearly every tile, and these stop at the first
 * changed word from each end) */
#define NIB 0xF0F0F0F0F0F0F0F0ull
static int first_chg(const uint8_t *sh, const uint8_t *g, int a, int b)
{
    for (; a + 8 <= b + 1; a += 8) {
        uint64_t x, y;
        memcpy(&x, sh + a, 8);
        memcpy(&y, g + a, 8);
        if ((x ^= y) & NIB) return a + __builtin_ctzll(x & NIB) / 8;
    }
    for (; a <= b; a++) if ((sh[a] ^ g[a]) >> 4) return a;
    return -1;
}
static int last_chg(const uint8_t *sh, const uint8_t *g, int a, int b)
{
    for (; b - 8 >= a - 1; b -= 8) {
        uint64_t x, y;
        memcpy(&x, sh + b - 7, 8);
        memcpy(&y, g + b - 7, 8);
        if ((x ^= y) & NIB) return b - __builtin_clzll(x & NIB) / 8;
    }
    for (; b >= a; b--) if ((sh[b] ^ g[b]) >> 4) return b;
    return -1;
}

#ifdef __aarch64__
#include <arm_neon.h>
/* 16 bytes' visible-change flags, 4 bits per byte (vshrn narrows the 0xFF lanes) */
static inline uint64_t nib_mask(uint8x16_t s, uint8x16_t g)
{
    uint8x16_t m = vtstq_u8(veorq_u8(s, g), vdupq_n_u8(0xF0));
    return vget_lane_u64(vreinterpret_u64_u8(vshrn_n_u16(vreinterpretq_u16_u8(m), 4)), 0);
}
#endif
/* first / last visibly changed i in [a, b], *i0 = -1: none. A whole 32-px tile
 * from two 16-byte NEON compares, else first_chg / last_chg (the
 * full-panel scan 7.96 -> 4.33 ms on a big core, 13.0 -> 7.9 on a little one,
 * box lists bit-exact, test/neon-bench) */
static inline void chg_pair(const uint8_t *sh, const uint8_t *g, int a, int b, int *i0, int *i1)
{
#ifdef __aarch64__
    if (b - a == 31) {
        uint64_t m0 = nib_mask(vld1q_u8(sh + a), vld1q_u8(g + a)), m1 = nib_mask(vld1q_u8(sh + a + 16), vld1q_u8(g + a + 16));
        if (!(m0 | m1)) { *i0 = -1; return; }
        *i0 = m0 ? a + __builtin_ctzll(m0) / 4 : a + 16 + __builtin_ctzll(m1) / 4;
        *i1 = m1 ? a + 16 + (63 - __builtin_clzll(m1)) / 4 : a + (63 - __builtin_clzll(m0)) / 4;
        return;
    }
#endif
    if ((*i0 = first_chg(sh, g, a, b)) >= 0) *i1 = last_chg(sh, g, *i0, b);
}

/* compare the frame with the shadow; add the pixels whose visible level (the
 * panel's 16 levels) differs to *out, as tight boxes per 32x32 tile merged with
 * their neighbours — NOT one bounding box: Xournal++ repaints stroke and
 * scrollbar in one damage rectangle, whose changed-pixel bbox spans from the
 * stroke to the screen edge. `all`: every pixel counts. Compared against `shadow`
 * in ordinary RAM, never by reading ebuf back: the /dev/ebc mapping is uncached
 * (a full-screen read-compare took ~800 ms). Row by row (a stream: the
 * A53's prefetcher; tile by tile, 32 strided rows per tile, was 5.2 vs 3.4 ms
 * per full window on the device), per tile only its first/last
 * changed pixel, straight from shadow + frame (~3.7 -> ~0.6 ms per
 * full-window update on the host). */
/* scan's three steps (the rows in parts, scan_job): the frame's tiles
 * cleared; rows [pa, pb) (whole tile rows, or the frame's: a tile is written
 * by one call only); the tiles into *out in (ty, tx) order */
static uint8_t tile_chg[TW * TH];       /* tiles a burst's scans flagged (commit_skip) */
static int commit_skip = 1;             /* (atomic) a burst's dithered commit leaves the others alone */
static void scan_reset(const frame *f)
{
    box b = f->b;
    int tx0 = b.x0 / TILE, tx1 = (b.x1 - 1) / TILE, ty0 = b.y0 / TILE, ty1 = (b.y1 - 1) / TILE;
    for (int ty = ty0; ty <= ty1; ty++)
        for (int tx = tx0; tx <= tx1; tx++) tilebox[ty * TW + tx] = (box){0};
}
static void scan_collect(const frame *f, boxlist *out)
{
    box b = f->b;
    int tx0 = b.x0 / TILE, tx1 = (b.x1 - 1) / TILE, ty0 = b.y0 / TILE, ty1 = (b.y1 - 1) / TILE;
    for (int ty = ty0; ty <= ty1; ty++)
        for (int tx = tx0; tx <= tx1; tx++)
            if (!box_empty(tilebox[ty * TW + tx])) {
                list_add(out, tilebox[ty * TW + tx]);
                tile_chg[ty * TW + tx] = 1;
            }
}
static void scan_rows(const frame *f, int all, int pa, int pb)
{
    box b = f->b;
    int ty0 = b.y0 / TILE, ty1 = (b.y1 - 1) / TILE;
    for (int px = pa; px < pb; px++) {
        const uint8_t *sh = shadow + (size_t)px * PW + f->bx0, *gr = f->g + (size_t)(px - b.x0) * f->s;
        box *trow = &tilebox[px / TILE];
        for (int ty = ty0; ty <= ty1; ty++) {
            /* this tile's rows: py in [lo, hi), i = b.y1 - 1 - py */
            int lo = ty * TILE > b.y0 ? ty * TILE : b.y0, hi = ty * TILE + TILE < b.y1 ? ty * TILE + TILE : b.y1;
            int ia = b.y1 - hi, ib = b.y1 - 1 - lo, i0 = ia, i1 = ib;
            if (!all) {
                chg_pair(sh, gr, ia, ib, &i0, &i1);
                if (i0 < 0) continue;
            }
            int y0 = b.y1 - 1 - i1, y1 = b.y1 - i0;
            box *t = trow + ty * TW;
            if (box_empty(*t)) *t = (box){ px, y0, px + 1, y1 };
            else {                              /* px only grows */
                t->x1 = px + 1;
                if (y0 < t->y0) t->y0 = y0;
                if (y1 > t->y1) t->y1 = y1;
            }
        }
    }
}
static void scan(const frame *f, int all, boxlist *out)
{
    scan_reset(f);
    scan_rows(f, all, f->b.x0, f->b.x1);
    scan_collect(f, out);
}

/* the ordered dither's thresholds, dith[row & 63][col & 63] (epd-dither.h:
 * 64 x 64 blue noise decorrelated from the stagger's groups, or the old 8 x 8
 * Bayer: dither_pattern=). The GPU commit (epd_ingest_dither) builds the same
 * table from the same header */
static int dith_pat = -1;
static uint8_t dith[64][64];
/* dith's rows across the panel, dith_row[r][c] = dith[r][c & 63]: commit's
 * threshold row is a pointer into it (it was refilled with ~11
 * memcpys per frame row, every commit) */
static uint8_t dith_row[64][PW];
/* the commit's pattern: an app's own (its profile's dither=, set around its
 * burst's commits), else dither_pattern; the burst a motion run or not */
static int cm_pat = -1, cm_motion;
static int cur_pattern(void) { return cm_pat >= 0 ? cm_pat : dither_pattern; }
/* the ordered table: dizzy's ordered stand-in (C_Q4, ebuf, the GPU) blue noise */
static int ord_pattern(void) { return cur_pattern() == EPD_DITHER_DIZZY ? EPD_DITHER_BLUE : cur_pattern(); }
static void dither_table(void)
{
    if (dith_pat == ord_pattern()) return;
    epd_dither_table(dith, dith_pat = ord_pattern());
    for (int r = 0; r < 64; r++)
        for (int c = 0; c < PW; c++) dith_row[r][c] = dith[r][c & 63];
}
static struct { unsigned long n, serial, gpu, gview; double ms, max, lk; unsigned long long px; } dzs;  /* (stats; lk: the part under buf_mu; gpu: dots made there, gview: of those, from the ingest's own texture) */

/* write the frame into shadow + ebuf; C_BW leaves grey targets (visibly changed
 * ones) at their old value. Returns how many were left. (The stock kernel's
 * way, with ebuf; mainline: commit below, the same per pixel, vectorized.) */
static long commit_ebuf(const frame *f, int all, int how)
{
    long left = 0;
    dither_table();
    for (int px = f->b.x0; px < f->b.x1; px++) {
        const uint8_t *gr = f->g + (size_t)(px - f->b.x0) * f->s;
        size_t o0 = (size_t)px * PW + f->bx0;
        for (int i = 0; i < f->w; i++) {
            uint8_t g = gr[i];
            size_t o = o0 + i;
            if (!all && shadow[o] == g) continue;
            if (how == C_BW && ((shadow[o] ^ g) >> 4) && !bw_level(g)) { left++; continue; }
            if (how == C_Q4 && !bw_level(g)) {     /* ordered dither to 0/85/170/255: DU4's levels */
                int s = g * 3, q = s / 255;
                if (s % 255 > dith[px & 63][(f->bx0 + i) & 63]) q++;
                g = (uint8_t)(q * 85);
                if (shadow[o] == g) continue;
            }
            if ((how == C_DITHER || how == C_THRESH) && !bw_level(g)) {
                g = g > (how == C_THRESH ? 127 : dith[px & 63][(f->bx0 + i) & 63]) ? 255 : 0;
                if (shadow[o] == g) continue;
            }
            shadow[o] = g;
            if (ebuf) ebuf[o] = 0xFF000000u | g << 16 | g << 8 | g;
        }
    }
    return left;
}

/* commit's rows, branch-free (byte masks: 0xff = true), so they vectorize;
 * shadow = what the pixel becomes */
#define M8(c) ((uint8_t)-(uint8_t)(c))
__attribute__((optimize("O3")))
static long commit_bw_row(uint8_t *restrict sh, const uint8_t *restrict g, int n)
{
    unsigned left = 0;
    for (int i = 0; i < n; i++) {
        uint8_t v = g[i], s = sh[i], hv = v >> 4;
        uint8_t keep = M8(((s ^ v) >> 4) != 0) & M8(hv != 0) & M8(hv != 15);
        left += keep & 1;
        sh[i] = (uint8_t)((s & keep) | (v & ~keep));
    }
    return left;
}
/* greys to q (C_Q4: ordered dither to 0/85/170/255, else black/white at thr);
 * !all: an unchanged pixel stays */
__attribute__((optimize("O3")))
static void commit_q4_row(uint8_t *restrict sh, const uint8_t *restrict g, const uint8_t *restrict thr, int n, uint8_t all)
{
    for (int i = 0; i < n; i++) {
        uint8_t v = g[i], s = sh[i], hv = v >> 4;
        uint16_t t3 = (uint16_t)(v * 3);
        uint16_t k = (uint16_t)((t3 >= 255) + (t3 >= 510) + (t3 >= 765));
        k = (uint16_t)(k + ((uint16_t)(t3 - k * 255) > thr[i]));
        uint8_t q = (uint8_t)(k * 85);
        uint8_t bw = M8(hv == 0) | M8(hv == 15), stay = M8(s == v) & (uint8_t)~all;
        uint8_t nv = (uint8_t)((v & bw) | (q & ~bw));
        sh[i] = (uint8_t)((s & stay) | (nv & ~stay));
    }
}
__attribute__((optimize("O3")))
static void commit_bw2_row(uint8_t *restrict sh, const uint8_t *restrict g, const uint8_t *restrict thr, int n, uint8_t all)
{
    for (int i = 0; i < n; i++) {
        uint8_t v = g[i], s = sh[i], hv = v >> 4;
        uint8_t q = M8(v > thr[i]);
        uint8_t bw = M8(hv == 0) | M8(hv == 15), stay = M8(s == v) & (uint8_t)~all;
        uint8_t nv = (uint8_t)((v & bw) | (q & ~bw));
        sh[i] = (uint8_t)((s & stay) | (nv & ~stay));
    }
}

/* commit_bw2_row with dizzy's dots l (levels 0 / 30, maybe | EPD_DZ_FIXED)
 * for the threshold (restrict: the plain loop in commit_dizzy didn't
 * vectorize, 16 ms a commit on the device) */
static void commit_dz_row(uint8_t *restrict sh, const uint8_t *restrict g, const uint8_t *restrict l, int n, uint8_t all)
{
    for (int i = 0; i < n; i++) {
        uint8_t v = g[i], s = sh[i], hv = v >> 4;
        uint8_t q = M8((l[i] & 0x7f) != 0);
        uint8_t bw = M8(hv == 0) | M8(hv == 15), stay = M8(s == v) & (uint8_t)~all;
        uint8_t nv = (uint8_t)((v & bw) | (q & ~bw));
        sh[i] = (uint8_t)((s & stay) | (nv & ~stay));
    }
}

/* dizzy's bands: the rect cut at every DZ_BAND-th panel row, each band its
 * own rect (error stays in it: the seams are rect edges, invisible), run on
 * the main thread + DZ_WORKERS workers, a band at a time from a counter. The
 * cuts are fixed panel rows: the dots don't depend on the thread count */
#define DZ_BAND 64
#define DZ_WORKERS 7                            /* (at most: dizzy_threads= picks how many run) */
#define DZ_FAST 3                               /* (workers 0..2 anywhere, as before; the rest on the slow cores) */
static int dz_threads = 8;                      /* dizzy_threads=N: the main thread + N - 1 workers */
static uint8_t dz_lv[256];                      /* grey -> dizzy's level, | EPD_DZ_FIXED for black and white */
static struct {
    pthread_mutex_t mu;
    pthread_cond_t go, done;
    unsigned gen;
    int started, busy;
    /* the job: g's grey rows (gs apart) into t as levels, then dizzy; ru
     * (or NULL): the rect's 64 x 64 panel tiles whose dots are reused (in t
     * already, fixed), rntx a row of it */
    uint8_t *t;
    const uint8_t *g, *ru;
    int gs, w, h, x0, y0, serial, nb, next, left, fail, rntx;
} dzj = { .mu = PTHREAD_MUTEX_INITIALIZER, .go = PTHREAD_COND_INITIALIZER, .done = PTHREAD_COND_INITIALIZER };
static int dz_set_threads(int n)
{
    pthread_mutex_lock(&dzj.mu);                /* (between jobs: a job's bands all get done either way) */
    dz_threads = n < 1 ? 1 : n > DZ_WORKERS + 1 ? DZ_WORKERS + 1 : n;
    n = dz_threads;
    pthread_mutex_unlock(&dzj.mu);
    return n;
}
static void dz_run_bands(void)                  /* (dzj.mu held on entry and exit) */
{
    while (dzj.next < dzj.nb) {
        int b = dzj.next++;
        int r0 = b ? b * DZ_BAND - (dzj.y0 % DZ_BAND) : 0, r1 = (b + 1) * DZ_BAND - (dzj.y0 % DZ_BAND);
        if (r1 > dzj.h) r1 = dzj.h;
        pthread_mutex_unlock(&dzj.mu);
        for (int r = r0; r < r1; r++) {          /* (the levels, black and white fixed: in the band, on its core) */
            const uint8_t *gr = dzj.g + (size_t)r * dzj.gs;
            uint8_t *l = dzj.t + (size_t)r * dzj.w;
            if (!dzj.ru) {
                for (int i = 0; i < dzj.w; i++) l[i] = dz_lv[gr[i]];
                continue;
            }
            const uint8_t *rr = dzj.ru + (size_t)(((dzj.y0 + r) >> 6) - (dzj.y0 >> 6)) * dzj.rntx;
            for (int i = 0; i < dzj.w;) {        /* (a tile at a time: reused ones are in l already) */
                int c = dzj.x0 + i, e = ((c >> 6) + 1) * 64 - dzj.x0;
                if (e > dzj.w) e = dzj.w;
                if (!rr[(c >> 6) - (dzj.x0 >> 6)])
                    for (int j = i; j < e; j++) l[j] = dz_lv[gr[j]];
                i = e;
            }
        }
        int rc = (dzj.serial ? epd_dizzy_serial : epd_dizzy)(dzj.t + (size_t)r0 * dzj.w, dzj.w, r1 - r0, dzj.w, dzj.x0, dzj.y0 + r0);
        pthread_mutex_lock(&dzj.mu);
        if (rc < 0) dzj.fail = 1;
        if (!--dzj.left) pthread_cond_broadcast(&dzj.done);
    }
}
/* the slow cores (cpu_capacity below the highest), for the workers past DZ_FAST */
static int dz_slow_cpus(cpu_set_t *set)
{
    int cap[16], n = 0, hi = 0, k = 0;
    CPU_ZERO(set);
    for (; n < 16; n++) {
        char p[64];
        snprintf(p, sizeof p, "/sys/devices/system/cpu/cpu%d/cpu_capacity", n);
        FILE *fp = fopen(p, "r");
        if (!fp) break;
        if (fscanf(fp, "%d", &cap[n]) != 1) cap[n] = 0;
        fclose(fp);
        if (cap[n] > hi) hi = cap[n];
    }
    for (int i = 0; i < n; i++) if (cap[i] < hi) { CPU_SET(i, set); k++; }
    return k;
}
static void *dz_worker(void *arg)
{
    int idx = (int)(intptr_t)arg;
    cpu_set_t set;
    if (idx >= DZ_FAST && dz_slow_cpus(&set)) pthread_setaffinity_np(pthread_self(), sizeof set, &set);
    pthread_mutex_lock(&dzj.mu);
    for (unsigned seen = 0;;) {
        while (dzj.gen == seen) pthread_cond_wait(&dzj.go, &dzj.mu);
        seen = dzj.gen;
        if (idx < dz_threads - 1) dz_run_bands();   /* (dizzy_threads=: the others sit this one out) */
    }
    return NULL;
}
/* g's greys (w x h, gs apart, panel position x0, y0) as dizzy's dots into t
 * (w apart), in bands; -1: out of memory */
static int dz_bands(uint8_t *t, const uint8_t *g, int gs, int w, int h, int x0, int y0, int serial, const uint8_t *ru, int rntx)
{
    pthread_mutex_lock(&dzj.mu);
    if (!dzj.started) {
        dzj.started = 1;
        for (int v = 0; v < 256; v++) dz_lv[v] = (uint8_t)((v * 30 + 127) / 255 | (bw_level((uint8_t)v) ? EPD_DZ_FIXED : 0));
        epd_dizzy_init();                       /* (its tables, before a worker can build them too) */
        for (int i = 0; i < DZ_WORKERS; i++) {
            pthread_t th;
            if (!pthread_create(&th, NULL, dz_worker, (void *)(intptr_t)i)) pthread_detach(th);
        }
    }
    dzj.ru = ru; dzj.rntx = rntx;
    dzj.t = t; dzj.g = g; dzj.gs = gs; dzj.w = w; dzj.h = h; dzj.x0 = x0; dzj.y0 = y0; dzj.serial = serial; dzj.fail = 0;
    dzj.nb = (y0 % DZ_BAND + h + DZ_BAND - 1) / DZ_BAND;
    dzj.next = 0;
    dzj.left = dzj.nb;
    if (dzj.nb > 1) { dzj.gen++; pthread_cond_broadcast(&dzj.go); }
    dz_run_bands();
    while (dzj.left) pthread_cond_wait(&dzj.done, &dzj.mu);
    int fail = dzj.fail;
    pthread_mutex_unlock(&dzj.mu);
    return fail ? -1 : 0;
}

/* C_DITHER with dizzy: the rect's greys as levels 0..30 dithered as a whole
 * (epd-dizzy.h: layered in motion runs, dizzy_still= otherwise; its black and
 * white fixed: they keep their value and take no error), in row bands on
 * dizzy_threads= cores (dz_bands), then written as commit_bw2_row does (black/white targets
 * as they are, !all: a pixel already equal stays). Error stays inside a band.
 * Returns -1 (nothing done) when out of memory */
/* the dots of a frame, made before buf_mu (dz_prep: the pen thread waits on
 * buf_mu, and dizzy takes ~10-50 ms): DZ_PREP frames, by their pixels and
 * place; commit_dizzy takes them, or makes them itself */
#define DZ_PREP 8
static struct { const uint8_t *g; box b; int bx0, w, serial, gpu, gview; uint8_t *lv; size_t cap; double ms; } dzp[DZ_PREP];

/* dizzy_reuse=on (default off, for a feel check): a
 * scroll's dots move with the content. dz_pg / dz_pd: the greys and the
 * shadow values dizzy's last commit at each panel pixel left (the main thread
 * only). A 64 x 64 panel tile of a new rect takes the dots from where its
 * content was (a shift along the panel row: a portrait scroll; 0: unmoved)
 * when its greys equal dz_pg there exactly and the shadow there still holds
 * dz_pd (no other commit, ink or flash since); those dots go in fixed, the
 * rest is dithered as before (the error stops at the reused dots). The shift:
 * exact matches of a few sample rows against dz_pg */
static uint8_t *dz_ru;
static size_t dz_rcap;
static struct { unsigned long tiles, reused, shifted; } dzr;   /* (stats) */
#define DZ_SHIFT_MAX 900
static int dz_shifts(const frame *f, int *cand, int max)
{
    int rows = f->b.x1 - f->b.x0, w = f->w, n = 0;
    if (w < 128 || rows < 4) return 0;
    int votes[8] = { 0 }, sv[8], ns = 0;
    for (int q = 1; q <= 8; q++) {                  /* (8 sample rows) */
        int r = rows * q / 9;
        const uint8_t *g = f->g + (size_t)r * f->s, *p = dz_pg + (size_t)(f->b.x0 + r) * PW;
        int flat = 1;
        for (int i = 1; i < w && flat; i++) flat = g[i] == g[0];
        if (flat) continue;                         /* (a flat row matches every shift) */
        for (int d = 1; d <= DZ_SHIFT_MAX; d++) {
            int hit = 0;
            for (int sg = -1; sg <= 1 && !hit; sg += 2) {
                int sh = d * sg, i0 = f->bx0 + sh < 0 ? -(f->bx0 + sh) : 0, i1 = f->bx0 + w + sh > PW ? PW - f->bx0 - sh : w;
                if (i1 - i0 < w / 2 || memcmp(g + i0, p + f->bx0 + sh + i0, (size_t)(i1 - i0))) continue;
                int j = 0;
                while (j < ns && sv[j] != sh) j++;
                if (j == ns && ns < 8) sv[ns++] = sh;
                if (j < ns) votes[j]++;
                hit = 1;                            /* (the smallest one a row) */
            }
            if (hit) break;
        }
    }
    for (int k = 0; k < max; k++) {                 /* (the most voted first) */
        int b = -1;
        for (int j = 0; j < ns; j++) if (votes[j] > 0 && (b < 0 || votes[j] > votes[b])) b = j;
        if (b < 0) break;
        cand[n++] = sv[b];
        votes[b] = 0;
    }
    return n;
}
/* the reused tiles' dots into lv (fixed), their map in dz_ru; returns the
 * map, NULL when nothing is reused. locked: buf_mu held already */
static const uint8_t *dz_take(const frame *f, uint8_t *lv, int locked, int *rntx)
{
    if (!dz_reuse) return NULL;
    if (!dz_pg) {
        dz_pg = calloc((size_t)PW * PH, 1);
        dz_pd = calloc((size_t)PW * PH, 1);
        if (!dz_pg || !dz_pd) { free(dz_pg); free(dz_pd); dz_pg = dz_pd = NULL; }
        return NULL;                                /* (nothing to reuse yet) */
    }
    int rows = f->b.x1 - f->b.x0, w = f->w;
    int tx0 = f->bx0 >> 6, tx1 = (f->bx0 + w + 63) >> 6, ty0 = f->b.x0 >> 6, ty1 = (f->b.x0 + rows + 63) >> 6;
    int ntx = tx1 - tx0;
    size_t nt = (size_t)ntx * (ty1 - ty0);
    if (nt > dz_rcap) {
        uint8_t *nr = realloc(dz_ru, nt);
        if (!nr) return NULL;
        dz_ru = nr;
        dz_rcap = nt;
    }
    memset(dz_ru, 0, nt);
    int cand[4] = { 0 }, nc = 1 + dz_shifts(f, cand + 1, 3);
    int *pick = malloc(nt * sizeof *pick);          /* (the tile's shift; INT_MIN: none) */
    if (!pick) return NULL;
    for (size_t t = 0; t < nt; t++) pick[t] = INT_MIN;
    for (int ty = ty0; ty < ty1; ty++)              /* (the greys: unlocked) */
        for (int tx = tx0; tx < tx1; tx++) {
            int r0 = (ty << 6) - f->b.x0, r1 = r0 + 64, c0 = (tx << 6) - f->bx0, c1 = c0 + 64;
            if (r0 < 0) r0 = 0;
            if (r1 > rows) r1 = rows;
            if (c0 < 0) c0 = 0;
            if (c1 > w) c1 = w;
            dzr.tiles++;
            for (int c = 0; c < nc; c++) {
                int sh = cand[c], a = f->bx0 + c0 + sh;
                if (a < 0 || f->bx0 + c1 + sh > PW) continue;
                int ok = 1;
                for (int r = r0; r < r1 && ok; r++)
                    ok = !memcmp(f->g + (size_t)r * f->s + c0, dz_pg + (size_t)(f->b.x0 + r) * PW + a, (size_t)(c1 - c0));
                if (ok) { pick[(size_t)(ty - ty0) * ntx + (tx - tx0)] = sh; break; }
            }
        }
    if (!locked) pthread_mutex_lock(&buf_mu);
    int any = 0;
    for (int ty = ty0; ty < ty1; ty++)              /* (the shadow: still our dots there?) */
        for (int tx = tx0; tx < tx1; tx++) {
            int sh = pick[(size_t)(ty - ty0) * ntx + (tx - tx0)];
            if (sh == INT_MIN) continue;
            int r0 = (ty << 6) - f->b.x0, r1 = r0 + 64, c0 = (tx << 6) - f->bx0, c1 = c0 + 64;
            if (r0 < 0) r0 = 0;
            if (r1 > rows) r1 = rows;
            if (c0 < 0) c0 = 0;
            if (c1 > w) c1 = w;
            size_t a = (size_t)(f->bx0 + c0 + sh);
            int ok = 1;
            for (int r = r0; r < r1 && ok; r++)
                ok = !memcmp(shadow + (size_t)(f->b.x0 + r) * PW + a, dz_pd + (size_t)(f->b.x0 + r) * PW + a, (size_t)(c1 - c0));
            if (!ok) continue;
            for (int r = r0; r < r1; r++) {
                const uint8_t *sr = shadow + (size_t)(f->b.x0 + r) * PW + a;
                uint8_t *l = lv + (size_t)r * w + c0;
                for (int i = 0; i < c1 - c0; i++) l[i] = (uint8_t)((sr[i] >= 128 ? 30 : 0) | EPD_DZ_FIXED);
            }
            dz_ru[(size_t)(ty - ty0) * ntx + (tx - tx0)] = 1;
            dzr.reused++;
            dzr.shifted += sh != 0;
            any = 1;
        }
    if (!locked) pthread_mutex_unlock(&buf_mu);
    free(pick);
    *rntx = ntx;
    return any ? dz_ru : NULL;
}
static int dz_make(const frame *f, int k, int locked)
{
    int rows = f->b.x1 - f->b.x0, w = f->w;
    size_t n = (size_t)rows * w;
    if (n > dzp[k].cap) {
        uint8_t *nl = realloc(dzp[k].lv, n);
        if (!nl) return -1;
        dzp[k].lv = nl;
        dzp[k].cap = n;
    }
    double t0 = tsec();
    int serial = !cm_motion && dizzy_still_serial, rntx = 0;
    dzp[k].g = NULL;
    const uint8_t *ru = dz_take(f, dzp[k].lv, locked, &rntx);
    dzp[k].gpu = dzp[k].gview = 0;
    if (dizzy_gpu && !serial && !ru && ing && gt_main()) {     /* (the GPU's: the rect whole, no bands) */
        if (!dzj.started) dz_bands(dzp[k].lv, f->g, f->s, 0, 0, 0, 0, 0, NULL, 0);   /* (dz_lv, once) */
        /* a wlsrc frame (sway: the greys are the ingest's own output, as commit_gpu tells them) needs no upload */
        int view = wl_is_view(f);
        epd_ingest_dizzy_form(ing, dizzy_form_layered ? EPD_GDZ_LAYERED : EPD_GDZ_BLOCK);
        dzp[k].gpu = epd_ingest_dizzy_v(ing, f->g, f->s, f->bx0, f->b.x0, w, rows, dz_lv, dzp[k].lv, w, 500, view) == 0;
        dzp[k].gview = dzp[k].gpu && epd_ingest_dizzy_src(ing);
    }
    if (!dzp[k].gpu && dz_bands(dzp[k].lv, f->g, f->s, w, rows, f->bx0, f->b.x0, serial, ru, rntx) < 0) return -1;
    dzp[k].g = f->g; dzp[k].b = f->b; dzp[k].bx0 = f->bx0; dzp[k].w = w; dzp[k].serial = serial;
    dzp[k].ms = (tsec() - t0) * 1e3;
    return 0;
}
/* frames fr[0..n) a C_DITHER commit will take: their dots now, unlocked */
static void dz_prep(const frame *fr, int n, int how)
{
    if (ebuf || how != C_DITHER || cur_pattern() != EPD_DITHER_DIZZY) return;
    for (int i = 0, k = 0; i < n && k < DZ_PREP; i++)
        if (fr[i].g && fr[i].b.x1 > fr[i].b.x0 && fr[i].w > 0) dz_make(&fr[i], k++, 0);
}
static void dz_prep_drop(void) { for (int k = 0; k < DZ_PREP; k++) dzp[k].g = NULL; }

static int commit_dizzy(const frame *f, int all)
{
    int rows = f->b.x1 - f->b.x0, w = f->w;
    size_t n = (size_t)rows * w;
    if (rows <= 0 || w <= 0) return 0;
    int serial = !cm_motion && dizzy_still_serial, k = 0;
    while (k < DZ_PREP && !(dzp[k].g == f->g && dzp[k].b.x0 == f->b.x0 && dzp[k].b.x1 == f->b.x1 &&
                            dzp[k].bx0 == f->bx0 && dzp[k].w == w && dzp[k].serial == serial)) k++;
    if (k == DZ_PREP && dz_make(f, k = 0, 1) < 0) return -1;   /* (not made ahead: now, under buf_mu) */
    const uint8_t *lv = dzp[k].lv;
    double ms0 = dzp[k].ms, t0 = tsec();
    dzp[k].g = NULL;                            /* (taken: once) */
    for (int r = 0; r < rows; r++) {
        uint8_t *sh = shadow + (size_t)(f->b.x0 + r) * PW + f->bx0;
        const uint8_t *gr = f->g + (size_t)r * f->s;
        commit_dz_row(sh, gr, lv + (size_t)r * w, w, M8(all));
        if (dz_pg) {                            /* (dizzy_reuse: what this commit left) */
            memcpy(dz_pg + (size_t)(f->b.x0 + r) * PW + f->bx0, gr, (size_t)w);
            memcpy(dz_pd + (size_t)(f->b.x0 + r) * PW + f->bx0, sh, (size_t)w);
        }
    }
    double lk = (tsec() - t0) * 1e3, ms = ms0 + lk;
    dzs.n++; dzs.serial += serial; dzs.gpu += dzp[k].gpu; dzs.gview += dzp[k].gview; dzs.ms += ms; dzs.lk += lk; dzs.px += n;
    if (ms > dzs.max) dzs.max = ms;
    return 0;
}

/* commit (see commit_ebuf), row by row and branch-free: the
 * per-pixel loops were ~3.5 ms per full-window update on the host, ~0.3 now */
/* commit_cpu's rows [pa, pb) (dither_table and thr127 made by the caller) */
static uint8_t thr127[PW];                     /* C_THRESH's threshold row: 127 */
static long commit_rows(const frame *f, int all, int how, int pa, int pb)
{
    long left = 0;
    for (int px = pa; px < pb; px++) {
        const uint8_t *gr = f->g + (size_t)(px - f->b.x0) * f->s;
        uint8_t *sh = shadow + (size_t)px * PW + f->bx0;
        if (how == C_BW) { left += commit_bw_row(sh, gr, f->w); continue; }
        if (how != C_Q4 && how != C_DITHER && how != C_THRESH) { memcpy(sh, gr, (size_t)f->w); continue; }
        const uint8_t *thr = how == C_THRESH ? thr127 : dith_row[px & 63] + f->bx0;   /* the dither's row */
        if (how == C_Q4) commit_q4_row(sh, gr, thr, f->w, M8(all));
        else commit_bw2_row(sh, gr, thr, f->w, M8(all));
    }
    return left;
}
static void commit_prep(void)
{
    if (thr127[0] != 127) memset(thr127, 127, sizeof thr127);
    dither_table();
}

/* ---- : the update job, update_job=pool ---------------
 * scan and commit of one frame in parts of whole tile rows (TILE panel rows:
 * a part owns its tiles in tilebox and its shadow rows), run by upd_run
 * (epd_stream_upd_run on the stream's workers, or epd-upd.h's own pool; the
 * caller takes parts too); the clear, the collect, the "left" sum and the
 * frame order stay on the caller, so the boxes, the shadow and the counts are
 * the one-pass bytes. NULL upd_run (update_job=main) or a frame below
 * UPD_MIN_PX: one pass, as before. Frames one after the other (two of a
 * burst may share a tile) */
static void (*upd_run)(void *ctx, void (*fn)(void *, int), void *arg, int n);
static void *upd_ctx;
#define UPD_MIN_PX (128 * 1024)
#define UPD_PARTS ((PH + TILE - 1) / TILE + 1)
struct upd_job { const frame *f; int all, how, t0, skip; long left[UPD_PARTS]; };
static void upd_part_rows(const struct upd_job *j, int k, int *a, int *b)
{
    *a = (j->t0 + k) * TILE;
    *b = *a + TILE;
    if (*a < j->f->b.x0) *a = j->f->b.x0;
    if (*b > j->f->b.x1) *b = j->f->b.x1;
}
/* commit_rows' C_DITHER rows [pa, pb) of one tile row, only in the tiles a scan of this burst
 * flagged: nothing visible differs in the others (the shadow keeps its low bits there), 45 % of
 * a Nautilus scroll's tiles. Neighbours merge into one span: longer vector runs */
static void commit_rows_flagged(const frame *f, int pa, int pb)
{
    const box b = f->b;
    const int tx = pa / TILE, ty0 = b.y0 / TILE, ty1 = (b.y1 - 1) / TILE;
    int sa[TH], sb[TH], ns = 0;
    for (int ty = ty1; ty >= ty0; ty--) {       /* (i grows as py falls) */
        if (!tile_chg[ty * TW + tx]) continue;
        int lo = ty * TILE > b.y0 ? ty * TILE : b.y0, hi = ty * TILE + TILE < b.y1 ? ty * TILE + TILE : b.y1;
        int ia = b.y1 - hi, ib = b.y1 - lo;
        if (ns && sb[ns - 1] == ia) sb[ns - 1] = ib;
        else { sa[ns] = ia; sb[ns++] = ib; }
    }
    for (int px = pa; px < pb; px++) {
        const uint8_t *gr = f->g + (size_t)(px - b.x0) * f->s, *thr = dith_row[px & 63] + f->bx0;
        uint8_t *sh = shadow + (size_t)px * PW + f->bx0;
        for (int k = 0; k < ns; k++) commit_bw2_row(sh + sa[k], gr + sa[k], thr + sa[k], sb[k] - sa[k], 0);
    }
}
static void upd_scan_part(void *arg, int k)
{
    struct upd_job *j = arg;
    int a, b;
    upd_part_rows(j, k, &a, &b);
    scan_rows(j->f, j->all, a, b);
}
static void upd_commit_part(void *arg, int k)
{
    struct upd_job *j = arg;
    int a, b;
    upd_part_rows(j, k, &a, &b);
    if (j->skip) { commit_rows_flagged(j->f, a, b); j->left[k] = 0; return; }
    j->left[k] = commit_rows(j->f, j->all, j->how, a, b);
}
static int upd_parts(const frame *f, struct upd_job *j)
{
    if (!upd_run || !f->g || f->b.x1 <= f->b.x0 || (long)(f->b.x1 - f->b.x0) * f->w < UPD_MIN_PX) return 0;
    j->f = f;
    j->t0 = f->b.x0 / TILE;
    return (f->b.x1 - 1) / TILE - j->t0 + 1;
}
static void scan_job(const frame *f, int all, boxlist *out)
{
    struct upd_job j;
    int n = upd_parts(f, &j);
    if (!n) { scan(f, all, out); return; }
    j.all = all;
    scan_reset(f);
    upd_run(upd_ctx, upd_scan_part, &j, n);
    scan_collect(f, out);
}
/* commit_cpu's plain rows in parts (the caller rules out ebuf and dizzy) */
static long commit_rows_job(const frame *f, int all, int how)
{
    struct upd_job j;
    int n = upd_parts(f, &j);
    commit_prep();
    if (!n) return commit_rows(f, all, how, f->b.x0, f->b.x1);
    j.all = all; j.how = how;
    j.skip = !all && how == C_DITHER && __atomic_load_n(&commit_skip, __ATOMIC_RELAXED);
    upd_run(upd_ctx, upd_commit_part, &j, n);
    long left = 0;
    for (int k = 0; k < n; k++) left += j.left[k];
    return left;
}

static long commit_cpu(const frame *f, int all, int how)
{
    if (ebuf) return commit_ebuf(f, all, how);  /* stock kernel: the ebuf writes */
    if (how == C_DITHER && cur_pattern() == EPD_DITHER_DIZZY && commit_dizzy(f, all) == 0) return 0;
    commit_prep();
    return commit_rows(f, all, how, f->b.x0, f->b.x1);
}

/* commit, and the shadow's GPU mirror kept equal (gtarget=): a view of wlsrc
 * (sway's frame) the ingest converted is committed there on the GPU too,
 * anything else marked for upload. Main thread, buf_mu */
static long commit(const frame *f, int all, int how)
{
    if (gt_mode && !gt_live && !gt_tried && ing && !ebuf && gt_main()) gt_setup();
    if (gt_live && (epd_ingest_dead(ing) || epd_stream_gpu_dead(eng))) gt_off("the GPU hung");
    if (!gt_live) return commit_cpu(f, all, how);
    gt_flush();                     /* (the GPU's commit reads the mirror) */
    long left = commit_cpu(f, all, how);
    epd_ingest_rect pr = { f->bx0, f->b.x0, f->w, f->h };
    epd_ingest_dither(ing, ord_pattern());
    int dz = how == C_DITHER && cur_pattern() == EPD_DITHER_DIZZY;  /* (dizzy: no GPU twin, uploaded) */
    if (!dz && wl_is_view(f) &&
        epd_ingest_mirror_commit(ing, pr, all, how) == 0) tmt.gcommits++;
    else { gt_mark(pr); tmt.ucommits++; }
    return left;
}

/* the burst's commit in parts where commit is the plain CPU rows (no
 * GPU mirror, no ebuf, no dizzy), else commit as before */
static long commit_pool(const frame *f, int all, int how)
{
    if (!upd_run || gt_mode || ebuf || (how == C_DITHER && cur_pattern() == EPD_DITHER_DIZZY)) return commit(f, all, how);
    return commit_rows_job(f, all, how);
}

/* the job's runners and the placement (update_job= ...): main = none; gold +
 * fifo = the stream's workers; else the own pool, pinned and scheduled as set */
static upd_pool upool;
static void upd_eng_run(void *c, void (*fn)(void *, int), void *arg, int n) { epd_stream_upd_run(c, fn, arg, n); }
static int cpu_class(cpu_set_t *set, int fast)
{
    int cap[64], n = 0, hi = 0, lo = 1 << 30, k = 0;
    CPU_ZERO(set);
    for (; n < 64; n++) {
        char p[64];
        snprintf(p, sizeof p, "/sys/devices/system/cpu/cpu%d/cpu_capacity", n);
        FILE *fp = fopen(p, "r");
        if (!fp) break;
        if (fscanf(fp, "%d", &cap[n]) != 1) cap[n] = 0;
        fclose(fp);
        if (cap[n] > hi) hi = cap[n];
        if (cap[n] < lo) lo = cap[n];
    }
    if (!n || hi == lo) return 0;           /* (one class: unpinned) */
    for (int i = 0; i < n; i++) if (fast ? cap[i] == hi : cap[i] < hi) { CPU_SET(i, set); k++; }
    return k;
}
static int cpu_list(const char *s, cpu_set_t *set)
{
    int k = 0;
    CPU_ZERO(set);
    while (s && *s) {
        char *e;
        long a = strtol(s, &e, 10), b = a;
        if (e == s) break;
        if (*e == '-') { s = e + 1; b = strtol(s, &e, 10); if (e == s) break; }
        for (long c = a; c <= b && c < CPU_SETSIZE; c++) if (c >= 0) { CPU_SET((int)c, set); k++; }
        if (*e != ',') break;
        s = e + 1;
    }
    return k;
}
static void upd_apply(void)
{
    const char *where = "-";
    if (!update_job) { upd_run = NULL; upd_ctx = NULL; where = "main thread"; }
    else if (eng && update_cpus == 0 && update_rt == 0) { upd_run = upd_eng_run; upd_ctx = eng; where = "the stream's workers"; }
    else {
        cpu_set_t c;
        int k = update_cpus == 2 ? cpu_list(update_list, &c) : cpu_class(&c, update_cpus == 0);
        upd_pool_config(&upool, k ? &c : NULL, k > 0, update_rt);
        upd_run = upd_pool_runner; upd_ctx = &upool;
        where = k ? "its own helpers, pinned" : "its own helpers, unpinned";
    }
    __atomic_store_n(&epd_stream_upd_fifo, update_job && update_rt == 0, __ATOMIC_RELAXED);
    if (eng) epd_stream_set_upd_runner(eng, upd_run, upd_ctx);
    fprintf(stderr, "glassine-epdd-desktop: update_job=%s update_cpus=%s%s update_rt=%s worker_rt=%s (%s)\n", update_job ? "pool" : "main",
            update_cpus == 0 ? "gold" : update_cpus == 1 ? "silver" : "", update_cpus == 2 ? update_list : "",
            update_rt ? "nice" : "fifo", __atomic_load_n(&epd_stream_worker_rt, __ATOMIC_RELAXED) ? "nice" : "fifo", where);
}

/* scan + commit everything (a GC16 of the result must follow) */
static void grab(Display *dpy, Window root, box b, int all, boxlist *out)
{
    double t0 = tsec();
    frame f = snap(dpy, root, b, src_fb);       /* the slow part: unlocked */
    double t1 = tsec();
    if (!f.g) return;
    pthread_mutex_lock(&buf_mu);
    if (out) scan(&f, all, out);
    commit(&f, all, C_ALL);
    pthread_mutex_unlock(&buf_mu);
    fbuf_put(f.g);
    tm.n++;
    tm.snap += t1 - t0;
    if (t1 - t0 > tm.snap_max) tm.snap_max = t1 - t0;
    tm.commit += tsec() - t1;
}

/* "srccheck": the same rect from both sources; they must agree pixel by pixel */
static void src_check(Display *dpy, Window root, int W, int H)
{
    box b = { 0, 0, W, H };
    int64_t t0 = now_ms();
    frame a = snap(dpy, root, b, 1);
    int64_t t1 = now_ms();
    frame x = snap(dpy, root, b, 0);
    int64_t t2 = now_ms();
    fprintf(stderr, "glassine-epdd-desktop: srccheck: full screen from fb0 %lld ms, XGetImage %lld ms\n",
            (long long)(t1 - t0), (long long)(t2 - t1));
    if (!a.g || !x.g || !fbmem) { fprintf(stderr, "glassine-epdd-desktop: srccheck: no fb0 or no X image\n"); fbuf_put(a.g); fbuf_put(x.g); return; }
    long diff = 0, n = (long)a.w * a.h;
    int fx = -1, fy = -1;
    for (long i = 0; i < n; i++)
        if (a.g[i] != x.g[i]) { if (!diff) { fy = (int)(i / a.w); fx = (int)(i % a.w); } diff++; }
    fprintf(stderr, "glassine-epdd-desktop: srccheck: fb0 vs XGetImage: %ld of %ld pixels differ%s", diff, n, diff ? "" : "\n");
    if (diff) fprintf(stderr, " (first at panel %d,%d)\n", a.bx0 + fx, fy);
    /* both as panel-order PGMs, for a look */
    for (int k = 0; k < 2; k++) {
        FILE *o = prof_out(k ? "glassine-srccheck-x.pgm" : "glassine-srccheck-fb.pgm");
        if (!o) continue;
        fprintf(o, "P5 %d %d 255\n", a.w, a.h);
        fwrite(k ? x.g : a.g, 1, (size_t)n, o);
        fclose(o);
    }
    fbuf_put(a.g); fbuf_put(x.g);
}

/* the pen preview's brush: diameter in panel px
 * from wmin (no pressure) to wmax (full), pressure^exp in between; BR_ERASE
 * whitens (an eraser that removes ink), BR_NONE draws nothing (the app's
 * frame shows it: light colours, highlighters, a tool that doesn't draw) */
enum { BR_UNSET, BR_NONE, BR_PEN, BR_ERASE };
typedef struct { uint8_t kind; float wmin, wmax, exp; } brush;
typedef struct { brush tip, rub; } inkbrush;       /* the pen's tip and its eraser end */
#define BRUSH_DEFAULT ((inkbrush){ { BR_PEN, 3, 7, 1 }, { BR_NONE, 0, 0, 1 } })

/* Brushes v3 (epd-brush.h): a prepared brush per brush file, cached by id.
 * inkbh = epdd's handle on one: refs (atomic) = one per mask slot that names
 * it (main thread) + one per pen stroke drawing with it (pen thread). The
 * main thread alone loads and frees: an entry no longer cached and at refs 0
 * is freed by bh_sweep (at "ink end" and on the pen thread's news). The two
 * discs (v2 lines, presets, the default) are permanent. */
typedef struct {
    char id[17];
    epd_brush *b;                       /* NULL: a bad/missing file (logged once) */
    int refs;                           /* atomic: slots + strokes */
    uint8_t cached, perm;
    int64_t used;                       /* LRU (main) */
} inkbh;
#define INK_NBH 160                     /* handles: 2 masks x <= 64 ids + strokes + the cache */
#define INK_NCACHE 24                   /* cached ids (each <= 64 KiB of tips) */
static inkbh ink_bh[INK_NBH], ink_disc_h[2];   /* [0] pen, [1] eraser */
/* one pen end of a mask slot: the brush (NULL: no preview) + the line's
 * diameter range / pressure exponent. Immutable once in a slot. */
typedef struct { inkbh *h; float dmin, dmax, exp; int gate; } inkend;   /* gate: INK_START_* px (0 = INK_START_PX) */
typedef struct { inkend tip, rub; } inkslot;
static int brush_id_ok(const char *s)
{
    for (int i = 0; i < 16; i++) if (!((s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'f'))) return 0;
    return s[16] == 0;
}
static void bh_hold(inkbh *h) { if (h && !h->perm) __atomic_add_fetch(&h->refs, 1, __ATOMIC_ACQ_REL); }
static void bh_drop(inkbh *h) { if (h && !h->perm) __atomic_sub_fetch(&h->refs, 1, __ATOMIC_ACQ_REL); }
/* free what nobody holds any more (main thread) */
static void bh_sweep(void)
{
    for (int i = 0; i < INK_NBH; i++) {
        inkbh *h = &ink_bh[i];
        if (!h->id[0] || h->cached || __atomic_load_n(&h->refs, __ATOMIC_ACQUIRE)) continue;
        if (h->b) epdb_unref(h->b);
        memset(h, 0, sizeof *h);
    }
}
static void ink_init(void)
{
    if (ink_disc_h[0].b) return;
    for (int e = 0; e < 2; e++) {
        ink_disc_h[e].b = epdb_disc(e);
        ink_disc_h[e].perm = 1;
        snprintf(ink_disc_h[e].id, sizeof ink_disc_h[e].id, e ? "disc-eraser" : "disc");
    }
}
/* the brush file of a valid id: cached, else read + prepared (main thread
 * only, at "ink end"); NULL when missing or bad (that id then falls back to
 * the line's disc, logged once while it stays cached) */
static inkbh *bh_get(const char *id)
{
    int64_t now = now_ms();
    inkbh *fr = NULL, *lru = NULL;
    int ncached = 0;
    for (int i = 0; i < INK_NBH; i++) {
        inkbh *h = &ink_bh[i];
        if (h->cached && !strcmp(h->id, id)) { h->used = now; return h->b ? h : NULL; }
        if (h->cached) { ncached++; if (!lru || h->used < lru->used) lru = h; }
        else if (!fr && !h->id[0]) fr = h;
    }
    /* (evicted: freed by the next bh_sweep once no slot/stroke holds it; no
     * sweep here: ink_rebuild may hold an unreffed handle from this call) */
    if (ncached >= INK_NCACHE && lru) lru->cached = 0;
    if (!fr) { vlog("ink brush %s: no free handle, disc\n", id); return NULL; }
    char path[64], err[96] = "";
    snprintf(path, sizeof path, NB_DIR "/%s.brush", id);
    static uint8_t buf[NB_HDR + NB_MAXTIP * NB_MAXTIP * NB_MAXVAR];
    size_t len = 0;
    struct stat st;
    int fd = open(path, O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) snprintf(err, sizeof err, "%s", strerror(errno));
    else if (fstat(fd, &st) || !S_ISREG(st.st_mode) || st.st_size < NB_HDR || st.st_size > (off_t)sizeof buf)
        snprintf(err, sizeof err, "not a brush file");
    else {
        while (len < (size_t)st.st_size) {
            ssize_t k = read(fd, buf + len, sizeof buf - len);
            if (k <= 0) break;
            len += (size_t)k;
        }
        if (len != (size_t)st.st_size) snprintf(err, sizeof err, "short read");
    }
    if (fd >= 0) close(fd);
    epd_brush *b = NULL;
    if (!err[0]) {
        char got[17];
        epdb_id(buf, len, got);
        if (strcmp(got, id)) snprintf(err, sizeof err, "content id %s", got);
        else b = epdb_load(buf, len, err, sizeof err);
    }
    if (!b) vlog("ink brush %s: %s (the line's disc instead)\n", id, err[0] ? err : "rejected");
    else vlog("ink brush %s: loaded (%zu bytes, kind %d)\n", id, len, epdb_kind(b));
    memset(fr, 0, sizeof *fr);
    snprintf(fr->id, sizeof fr->id, "%s", id);
    fr->b = b;
    fr->cached = 1;
    fr->used = now;
    return b ? fr : NULL;
}

/* ---- direct pen ink --------------------------------------------------------
 * Draw the stroke into the panel buffer straight from the
 * pen's evdev events and DU it at once, instead of waiting for the app to render
 * it and X to report the damage. The app's real rendering replaces the preview
 * (GC16 of the inked area once drawing pauses). Only for windows whose WM_CLASS
 * profile has ink=on: the daemon cannot tell drawing from tapping a menu, and does
 * not know the app's brush (preview = black, width from pressure). No preview
 * for the eraser tool or with a side button held.
 * Pen devices: every evdev whose name starts with "Glassine Pen" (glassine-penfix's, stock
 * kernel), or the mainline i2c-hid Wacom node with pressure ("... 2D1F:012B ..."). */
#define MAXPEN 4
static struct pen {
    int fd, minx, maxx, miny, maxy, maxp;
    int flip;                              /* mainline i2c-hid: axes turned 180 degrees */
    int x, y, p, touch, tool_pen, rubber, btn;
    int tx, ty, has_tilt;                  /* ABS_TILT_X/Y (raw) */
    int tmin[2], tmax[2], tres[2];         /* their absinfo */
    int lx, ly, sx, sy, drawing;           /* last drawn point, stroke start */
    inkend br;                             /* the stroke's brush (from its start; holds a ref) */
    epdb_stroke st;                        /* the renderer's stroke state */
    epdb_point sp;                         /* the stroke's first point */
    int64_t inked_ms;                      /* last preview drawn (hover hold) */
} pens[MAXPEN];
/* the pen must travel this far (|dx|+|dy|, panel px) before a stroke inks: a
 * tap on a button stays clean. Inside areas an app registered (protocol, GTK
 * module, discovery) there are no buttons: the stroke inks as soon as the
 * pen moved 1 px (a tap without movement still leaves nothing;
 * step 1) */
#define INK_START_PX 6
#define INK_START_SDK 1
/* handwriting lifts the tip between strokes: X's repaints (a ~100 ms snapshot
 * that blocks this loop, then updates over the fresh ink) wait while the pen
 * that just inked still hovers, up to this long after its last ink */
#define INK_HOVER_MS 1500
static int npens;

static void pen_scan(void)
{
    for (int i = 0; i < npens; i++) { close(pens[i].fd); bh_drop(pens[i].br.h); pens[i].br.h = NULL; }
    npens = 0;
    DIR *d = opendir("/dev/input");
    if (!d) return;
    struct dirent *e;
    while ((e = readdir(d)) && npens < MAXPEN) {
        if (strncmp(e->d_name, "event", 5)) continue;
        char path[300], name[128] = "";
        snprintf(path, sizeof path, "/dev/input/%s", e->d_name);
        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) continue;
        ioctl(fd, EVIOCGNAME(sizeof name), name);
        /* stock: glassine-penfix's "Glassine Pen"; mainline: the Wacom digitizer via
         * i2c-hid ("... 2D1F:012B ..."), only its node with pressure */
        unsigned long absb[(ABS_MAX + 1 + 8 * sizeof(long) - 1) / (8 * sizeof(long))] = { 0 };
        ioctl(fd, EVIOCGBIT(EV_ABS, sizeof absb), absb);
        int has_p = (absb[ABS_PRESSURE / (8 * sizeof(long))] >> (ABS_PRESSURE % (8 * sizeof(long)))) & 1;
        if (strncmp(name, "Glassine Pen", 12) && !(strstr(name, "2D1F:012B") && has_p)) { close(fd); continue; }
        struct pen *p = &pens[npens++];
        memset(p, 0, sizeof *p);
        struct input_absinfo a;
        p->fd = fd;
        p->sx = -1;
        /* the mainline Wacom node reports the stock axes turned 180 degrees
         * (top-left tap = raw X~560/Y~15170; X matrix 0 -1 1 1 0 0) */
        p->flip = strstr(name, "2D1F:012B") != NULL;
        ioctl(fd, EVIOCGABS(ABS_X), &a); p->minx = a.minimum; p->maxx = a.maximum;
        ioctl(fd, EVIOCGABS(ABS_Y), &a); p->miny = a.minimum; p->maxy = a.maximum;
        ioctl(fd, EVIOCGABS(ABS_PRESSURE), &a); p->maxp = a.maximum > 0 ? a.maximum : 1;
        /* tilt (both nodes forward it: glassine-penfix copies the digitizer's ranges) */
        int has_tx = (absb[ABS_TILT_X / (8 * sizeof(long))] >> (ABS_TILT_X % (8 * sizeof(long)))) & 1;
        int has_ty = (absb[ABS_TILT_Y / (8 * sizeof(long))] >> (ABS_TILT_Y % (8 * sizeof(long)))) & 1;
        if (has_tx && has_ty && !ioctl(fd, EVIOCGABS(ABS_TILT_X), &a) && a.maximum > a.minimum) {
            p->tmin[0] = a.minimum; p->tmax[0] = a.maximum; p->tres[0] = a.resolution;
            if (!ioctl(fd, EVIOCGABS(ABS_TILT_Y), &a) && a.maximum > a.minimum) {
                p->tmin[1] = a.minimum; p->tmax[1] = a.maximum; p->tres[1] = a.resolution;
                p->has_tilt = 1;
            }
        }
        int clk = CLOCK_MONOTONIC;          /* event times for the brush's velocity */
        ioctl(fd, EVIOCSCLOCKID, &clk);
        if (verbose > 1) fprintf(stderr, "glassine-epdd-desktop: pen %s \"%s\" tilt %s %d..%d res %d\n", path, name,
                                 p->has_tilt ? "yes" : "no", p->tmin[0], p->tmax[0], p->tres[0]);
    }
    closedir(d);
}

/* ---- per-app profiles (E-ink Center, /etc/glassine/eink.conf) ------------------
 * One line per app: MATCH mode=auto|quality|fast touch=auto|quality|fast ink=on|off
 * MATCH = substring of the lower-cased "WM_CLASS class + name"; "*" = every
 * other app (and the desktop/panel). First match wins. Lines starting with "@"
 * are control commands (e.g. "@cut=auto", "@dither=on").
 * mode: repaints the app makes on its own — auto: sized by area (DU small,
 * GC16 medium/large, motion runs fast); quality: always GC16 (no DU
 * ghosting, slower); fast: dithered DU with the frame cut ("X mode"),
 * one GC16 cleanup when it settles
 * touch: during a gesture (epd-motion: + MO_HOLD_MS after the lift): auto = as mode
 * ink: direct pen ink (see below)
 * brush=W | brush=WMIN-WMAX[^EXP] | brush=none the pen preview's stroke:
 * diameter in panel px (~9 px/mm), WMIN without pressure to WMAX at
 * full, pressure^EXP between (default 3-7^1); an app's own brush
 * (glassine_ink_v1 set_brush) wins over it
 * eraser=W | eraser=WMIN-WMAX[^EXP] | eraser=none the pen's eraser end:
 * whitens at that width (default none: the app's frame shows it)
 * paint=LO-HI | paint=STEP the app's own run range (mmin=/mmax= names, e.g.
 * paint=a2-a2_4; STEP alone = fixed): glassine-settings > Apps "Own
 * responsiveness"; without it the global @mmin/@mmax. The app's
 * stills paint at LO too (still=range), LO == HI = one mode for
 * everything (no cleanups; the engine's DC healing still runs)
 * curve=on|off with paint=: the app's Gradual speed-up (off: its stills at
 * LO, any motion at HI); without it the global @curve
 * busy=auto|on|off its own repaints as motion (video players): auto (default)
 * = detected by rate and area (@busy=F), on = every repaint keeps its run
 * going without input, off = never (an app whose steady output should
 * not hold back the cleanup)
 * dither=blue|bayer|dizzy the app's own dither pattern (dither_pattern=);
 * without it the global one
 * motion=always every repaint of the app is motion at its fluidity (its
 * paint= range's fast end, else the global one), with or without input;
 * the cleanup only once it is quiet (games: a slow scene stays A2, see
 * ALWAYS in epd-policy.h). glassine-settings > Apps > Content "Moving"
 * pages=on|off|N the app replaces its page instead of moving it (readers:
 * KOReader): the finger alone is no motion, only a measured content
 * shift; a page change is a GC16 partial, every Nth (on = 6) a flash.
 * Default off. See PAGES in epd-policy.h. glassine-settings > Apps > Content "Pages"
 * An app line without mode= or touch= takes the '*' line's (glassine-settings
 * writes only what an app changes).
 * The profile of a burst = the app window under its largest changed area
 * (topmost of _NET_CLIENT_LIST_STACKING). Seen classes go to /run/glassine-epdd-desktop.classes
 * for the E-ink Center. Re-read when the file changes. */
#define PROF_CONF "/etc/glassine/eink.conf"
#define CLASSES "/run/glassine-epdd-desktop.classes"
#define DITHER_OFF (-2)                 /* a profile's dither=off */
typedef struct { char match[64]; int mode, touch, ink, lo, hi, curve, busy, dither; inkbrush br; int always, pages; } profile;   /* lo/hi: paint= (LV_*), -1 = the global range; always: motion=always; pages: pages= (0 off, else flash every Nth); curve= 0/1, -1 = global; busy: BUSY_*; dither: EPD_DITHER_*, -1 = dither_pattern, DITHER_OFF; br: brush=/eraser= (BR_UNSET, 0: the default) */
#define MAXPROF 48
static profile profs[MAXPROF], defprof = { "*", PM_AUTO, PM_FAST, 0, -1, -1, -1, BUSY_AUTO, -1 };
/* the shell's own surfaces (bar, popups; the "shell" hints of glassine-shell-wl):
 * calm (burst_in.calm), not an eink.conf profile */
static profile shellprof = { "glassine-shell", PM_AUTO, PM_AUTO, 0, -1, -1, -1, BUSY_OFF, -1 };
static int nprof;
/* eink.conf's last read: mtime (ns) + size, and a second read 300 ms after a
 * change. The writers (glassine-settings, glassine-shell) rewrite it in place, key by
 * key: a read may land between two writes in the same mtime tick, or in a
 * truncated file (mmax=a2_2 written right after a read stayed
 * a2_4 until a reload, so motion never reached A2/2) */
static struct timespec prof_mt = { -2, 0 };
static void ink_rebuild(void);
static off_t prof_size = -1;
static int64_t prof_recheck;

static int pm_parse(const char *v) { return !strncmp(v, "quality", 7) ? PM_QUALITY : !strncmp(v, "fast", 4) ? PM_FAST : PM_AUTO; }
static const char *pm_name(int m) { return m == PM_QUALITY ? "quality" : m == PM_FAST ? "fast" : m == PM_PRESS ? "press" : "auto"; }

/* paint=LO-HI or paint=STEP (fixed): the run steps as mmin=/mmax= name them */
static void paint_parse(const char *v, int *lo, int *hi)
{
    static const char *const nm[LV_N] = { "gc16", "du4", "a2", "a2_8", "a2_6", "a2_4", "a2_2" };
    int r[2] = { -1, -1 };
    for (int k = 0; k < 2 && *v; k++) {
        size_t n = strcspn(v, "-");
        for (int i = 0; i < LV_N; i++)
            if (strlen(nm[i]) == n && !strncmp(v, nm[i], n)) r[k] = i;
        v += n;
        if (*v == '-') v++;
        else break;
    }
    if (r[1] < 0) r[1] = r[0];
    if (r[0] < 0 || r[1] < r[0]) return;
    *lo = r[0]; *hi = r[1];
}

/* brush=W | WMIN-WMAX[^EXP] | none */
static void brush_parse(const char *v, int kind, brush *b)
{
    float lo, hi, e = 1;
    int n = 0;
    if (!strcmp(v, "none")) { *b = (brush){ BR_NONE, 0, 0, 1 }; return; }
    if (sscanf(v, "%f-%f%n", &lo, &hi, &n) == 2) ;
    else if (sscanf(v, "%f%n", &lo, &n) == 1) hi = lo;
    else return;
    if (v[n] == '^') e = strtof(v + n + 1, NULL);
    if (lo < 0.5f) lo = 0.5f;
    if (hi < lo) hi = lo;
    if (hi > 200) hi = 200;
    if (!(e > 0.05f && e < 20)) e = 1;
    *b = (brush){ kind, lo, hi, e };
}

/* eink.conf's "@claim_state=arrays|packed|check" : the
 * engine's per-pixel state is chosen when it opens, before load_profiles
 * runs, so the line is read here first; the last one wins and it overrides
 * GLASSINE_CLAIM_STATE (set again at every start, so a reexec takes the file's
 * current value). No line: the service's env, else packed (and stagger_state
 * lanes; the engine's own default stays
 * arrays/runs for the tests) */
static void claim_state_conf(void)
{
    FILE *f = fopen(PROF_CONF, "r");
    char line[256], v[16] = "", g[16] = "";
    while (f && fgets(line, sizeof line, f)) {
        if (!strncmp(line, "@claim_state=", 13)) sscanf(line + 13, "%15[a-z]", v);
        if (!strncmp(line, "@stagger_state=", 15)) sscanf(line + 15, "%15[a-z]", g);
    }
    if (f) fclose(f);
    /* (a reexec keeps the environment, so a removed line would keep the old
     * value: the service's own values are saved at the first start
     * (GLASSINE_*_STATE0, "" = none) and put back when the line is gone) */
    static const char *const ev[2] = { "GLASSINE_CLAIM_STATE", "GLASSINE_STAGGER_STATE" };
    for (int i = 0; i < 2; i++) {
        char k0[32];
        snprintf(k0, sizeof k0, "%s0", ev[i]);
        if (!getenv(k0)) setenv(k0, getenv(ev[i]) ? getenv(ev[i]) : "", 1);
        const char *o = getenv(k0);
        if (!*(i ? g : v)) setenv(ev[i], *o ? o : i ? "lanes" : "packed", 1);   /* (the daemon's default: packed + lanes) */
    }
    if (!strcmp(v, "arrays") || !strcmp(v, "packed") || !strcmp(v, "check")) setenv("GLASSINE_CLAIM_STATE", v, 1);
    else if (*v) fprintf(stderr, "glassine-epdd-desktop: eink.conf: @claim_state=%s: not arrays|packed|check\n", v);
    /* (the same for @stagger_state=lanes|runs; lanes needs claim_state=packed) */
    if (!strcmp(g, "lanes") || !strcmp(g, "runs")) setenv("GLASSINE_STAGGER_STATE", g, 1);
    else if (*g) fprintf(stderr, "glassine-epdd-desktop: eink.conf: @stagger_state=%s: not lanes|runs\n", g);
}

static int load_profiles(void)
{
    struct stat st;
    if (stat(PROF_CONF, &st) < 0) {
        /* first run: defaults, editable by the desktop user (the panel group, epd-group.h) */
        mkdir("/etc/glassine", 0755);
        FILE *f = fopen(PROF_CONF, "w");
        if (f) {
            fputs("# glassine-epdd-desktop app refresh profiles — edited by the E-ink Center (glassine-eink-center)\n"
                  "# MATCH mode=auto|quality|fast touch=auto|quality|fast ink=on|off\n"
                  "xournalpp mode=auto touch=auto ink=on\n"
                  "luajit mode=auto touch=auto ink=off\n"
                  "* mode=auto touch=fast ink=off\n", f);
            fchown(fileno(f), 0, epd_group(0));
            fchmod(fileno(f), 0664);
            fclose(f);
        }
        if (stat(PROF_CONF, &st) < 0) { st.st_mtim = (struct timespec){ -1, 0 }; st.st_size = 0; }
    }
    int64_t now = now_ms();
    int same = st.st_mtim.tv_sec == prof_mt.tv_sec && st.st_mtim.tv_nsec == prof_mt.tv_nsec && st.st_size == prof_size;
    if (same && !(prof_recheck && now >= prof_recheck)) return 0;
    prof_recheck = same ? 0 : now + 300;      /* a change: read it again once it has settled */
    prof_mt = st.st_mtim;
    prof_size = st.st_size;
    nprof = 0;
    defprof = (profile){ "*", PM_AUTO, PM_FAST, 0, -1, -1, -1, BUSY_AUTO, -1 };
    FILE *f = fopen(PROF_CONF, "r");
    if (!f) return 1;
    char line[256], globs[1024] = "";     /* the effective "@" lines, logged below (the file */
    int nglob = 0, star = 0;                /* lost every "@" line and the stagger went off unnoticed) */
    size_t gn = 0;
    while (fgets(line, sizeof line, f)) {
        char *l = line;
        while (isspace((unsigned char)*l)) l++;
        if (!*l || *l == '#') continue;
        if (*l == '@') {
            nglob++;
            if (gn < sizeof globs) gn += snprintf(globs + gn, sizeof globs - gn, " %.*s", (int)strcspn(l + 1, " \t\n"), l + 1);
        }
        /* one owner per setting (user): the touch boost belongs to
         * the power mode (glassine-ddr -> GLASSINE_EPDD / the ctl FIFO); an old "@boost="
         * line here would win over it at every reload, so it is ignored */
        if (*l == '@' && !strncmp(l + 1, "boost=", 6)) continue;
        if (*l == '@') { apply_cmd(l + 1); continue; }
        profile pr = { "", -1, -1, 0, -1, -1, -1, BUSY_AUTO, -1 };  /* mode/touch -1: the default's */
        char *tok = strtok(l, " \t\n");
        if (!tok) continue;
        snprintf(pr.match, sizeof pr.match, "%s", tok);
        for (char *c = pr.match; *c; c++) *c = tolower((unsigned char)*c);
        while ((tok = strtok(NULL, " \t\n"))) {
            if (!strncmp(tok, "mode=", 5)) pr.mode = pm_parse(tok + 5);
            else if (!strncmp(tok, "touch=", 6)) pr.touch = pm_parse(tok + 6);
            else if (!strncmp(tok, "ink=", 4)) pr.ink = !strcmp(tok + 4, "on");
            else if (!strncmp(tok, "paint=", 6)) paint_parse(tok + 6, &pr.lo, &pr.hi);
            else if (!strncmp(tok, "curve=", 6)) pr.curve = strcmp(tok + 6, "off") != 0;
            else if (!strncmp(tok, "brush=", 6)) brush_parse(tok + 6, BR_PEN, &pr.br.tip);
            else if (!strncmp(tok, "eraser=", 7)) brush_parse(tok + 7, BR_ERASE, &pr.br.rub);
            else if (!strcmp(tok, "motion=always")) pr.always = 1;
            else if (!strncmp(tok, "pages=", 6)) pr.pages = !strcmp(tok + 6, "on") ? PAGES_FLASH : atoi(tok + 6) > 0 ? atoi(tok + 6) : 0;
            else if (!strncmp(tok, "busy=", 5)) pr.busy = !strcmp(tok + 5, "on") ? BUSY_ON : !strcmp(tok + 5, "off") ? BUSY_OFF : BUSY_AUTO;
            else if (!strncmp(tok, "dither=", 7)) pr.dither = !strncmp(tok + 7, "off", 3) ? DITHER_OFF : epd_dither_parse(tok + 7);
        }
        if (!strcmp(pr.match, "*")) {
            star = 1;
            if (pr.mode < 0) pr.mode = PM_AUTO;
            if (pr.touch < 0) pr.touch = PM_FAST;
            pr.lo = pr.hi = pr.curve = -1;  /* the global range is @mmin/@mmax, @curve */
            defprof = pr;
        } else if (nprof < MAXPROF) profs[nprof++] = pr;
    }
    fclose(f);
    for (int i = 0; i < nprof; i++) {       /* the '*' line may come last */
        if (profs[i].mode < 0) profs[i].mode = defprof.mode;
        if (profs[i].touch < 0) profs[i].touch = defprof.touch;
    }
    fprintf(stderr, "glassine-epdd-desktop: %d app profiles, default mode=%s touch=%s\n", nprof,
            pm_name(defprof.mode), pm_name(defprof.touch));
    fprintf(stderr, "glassine-epdd-desktop: eink.conf: %d @ lines,%s; '*' %s\n", nglob, nglob ? globs : " NONE (code defaults)", star ? "present" : "ABSENT");
    ink_rebuild();                          /* an app's ink= may have changed */
    return 1;
}

static const profile *profile_for(const char *cls)
{
    for (int i = 0; i < nprof; i++)
        if (strstr(cls, profs[i].match)) return &profs[i];
    return &defprof;
}

/* publish a class for the E-ink Center (unique lines) */
static void class_publish(const char *cls)
{
    char line[160]; int seen = 0;
    size_t l = strlen(cls);
    FILE *f = fopen(CLASSES, "r");
    if (f) {
        while (!seen && fgets(line, sizeof line, f))
            seen = !strncmp(line, cls, l) && line[l] == '\n';
        fclose(f);
    }
    if (!seen && cls[0] && cls[0] != ' ' && (f = fopen(CLASSES, "a"))) {
        fprintf(f, "%s\n", cls);
        fclose(f);
    }
}

/* window -> lower-cased "class name" cache; new classes are appended to CLASSES */
#define MAXWIN 64
static struct { Window w; char cls[128]; } wcache[MAXWIN];
static int nwcache;
static const char *win_class(Display *dpy, Window w)
{
    for (int i = 0; i < nwcache; i++) if (wcache[i].w == w) return wcache[i].cls;
    int i = nwcache < MAXWIN ? nwcache++ : (int)(w % MAXWIN);
    wcache[i].w = w;
    wcache[i].cls[0] = 0;
    XClassHint ch;
    if (XGetClassHint(dpy, w, &ch)) {
        snprintf(wcache[i].cls, sizeof wcache[i].cls, "%s %s", ch.res_class ? ch.res_class : "",
                 ch.res_name ? ch.res_name : "");
        for (char *c = wcache[i].cls; *c; c++) *c = tolower((unsigned char)*c);
        XFree(ch.res_name); XFree(ch.res_class);
        class_publish(wcache[i].cls);
    }
    return wcache[i].cls;
}

/* the client window with WM_CLASS in/under a top-level (xfwm4 frame -> client) */
static Window find_client(Display *dpy, Window w, int depth)
{
    XClassHint ch;
    if (XGetClassHint(dpy, w, &ch)) {
        XFree(ch.res_name); XFree(ch.res_class);
        return w;
    }
    if (depth <= 0) return None;
    Window r, par, *kids = NULL, found = None;
    unsigned int n = 0;
    if (!XQueryTree(dpy, w, &r, &par, &kids, &n)) return None;
    for (int i = (int)n - 1; i >= 0 && !found; i--) found = find_client(dpy, kids[i], depth - 1);
    if (kids) XFree(kids);
    return found;
}

/* profile of the topmost window containing portrait point (x,y): the root's
 * children in X stacking order — includes override-redirect windows (KOReader's
 * SDL window, menus), which _NET_CLIENT_LIST_STACKING leaves out */
static const profile *profile_at(Display *dpy, Window root, int x, int y, box *win)
{
    const profile *pr = &defprof;
    *win = (box){ 0, 0, PH, PW };                /* no app window: the whole screen */
    Window r, par, *kids = NULL;
    unsigned int n = 0;
    if (!XQueryTree(dpy, root, &r, &par, &kids, &n)) return pr;
    for (int i = (int)n - 1; i >= 0; i--) {
        XWindowAttributes wa;
        if (!XGetWindowAttributes(dpy, kids[i], &wa) || wa.map_state != IsViewable || wa.class == InputOnly) continue;
        /* with the X border, like the window-change boxes (top_get): without it
         * the flash of a newly shown window missed its 4 px border strip, so its
         * stale area stayed and the cleanup flashed all of it again (the
         * overview/app grid took 3 GC16 passes, ~2 s, to settle) */
        int bw2 = 2 * wa.border_width;
        if (x < wa.x || x >= wa.x + wa.width + bw2 || y < wa.y || y >= wa.y + wa.height + bw2) continue;
        Window c = find_client(dpy, kids[i], 2);
        if (!c) continue;               /* unnamed helper window: look below */
        pr = profile_for(win_class(dpy, c));
        *win = box_clip((box){ wa.x, wa.y, wa.x + wa.width + bw2, wa.y + wa.height + bw2 }, (box){ 0, 0, PH, PW });
        break;
    }
    if (kids) XFree(kids);
    return pr;
}

/* is the active window an ink app? (re-read on _NET_ACTIVE_WINDOW changes) */
static int ink_window;
static void check_active(Display *dpy, Window root)
{
    static Atom act;
    if (!act) act = XInternAtom(dpy, "_NET_ACTIVE_WINDOW", False);
    ink_window = 0;
    Atom type; int fmt; unsigned long n, after; unsigned char *data = NULL;
    if (XGetWindowProperty(dpy, root, act, 0, 1, False, XA_WINDOW, &type, &fmt, &n, &after, &data) != Success)
        return;
    Window w = (n == 1 && data) ? *(Window *)data : None;
    if (data) XFree(data);
    if (w) {
        const char *cls = win_class(dpy, w);
        ink_window = profile_for(cls)->ink;
        if (verbose) fprintf(stderr, "active window %s -> ink %d\n", cls, ink_window);
    }
}

/* Wayland source: the windows glassine-swayd reports (apps/app/active FIFO lines,
 * portrait px, bottom to top) stand in for the X tree */
static box wl_area;                     /* the workspace area: above it, the bar */
static char wl_active[128];

static void wl_check_active(void)
{
    ink_window = wl_active[0] ? profile_for(wl_active)->ink : 0;
    if (verbose) fprintf(stderr, "active window %s -> ink %d\n", wl_active[0] ? wl_active : "-", ink_window);
}

/* Ink areas (sway patch 0007,: sway sends each
 * window's visible region -- the window minus every surface above it (menus,
 * the shell, the keyboard, other windows) -- in portrait px:
 * ink begin / ink S X Y W H APP (one per rect; S 1 = the app registered it) / ink end
 * and with the app brushes (glassine_ink_v1 v2 set_brush, widths in panel px):
 * ink brush K TKIND TMIN TMAX TEXP RKIND RMIN RMAX REXP (KIND 0 unset 1 none 2 pen 3 erase)
 * ink r K S X Y W H APP (a rect with brush K, 0 = none)
 * The rects of apps with ink=on (or S 1) become a mask of 4x4 px cells (a cell
 * counts when its centre is inside); the pen thread tests each point against
 * the current mask (two buffers, swapped at "ink end", no lock). */
#define INK_C 4
#define INK_CW (PH / INK_C)
#define INK_CH (PW / INK_C)
#define INK_MAXR 256
static struct { box b; int sdk, br; char app[128]; } ink_r[INK_MAXR], ink_nr[INK_MAXR];   /* br: sway's brush (0: none) */
#define INK_MAXB 32
/* sway's brushes, 1-based: the v2 numbers + the v3 brush file ids ("" none) */
typedef struct { inkbrush v; char tip[17], rub[17]; } swbrush;
static swbrush ink_b[INK_MAXB + 1], ink_nb[INK_MAXB + 1];
static int ink_n, ink_nn = -1;          /* the areas in force / being received (-1: none) */
static uint8_t ink_mask[2][INK_CW * INK_CH];   /* a brush slot per cell, 0 = no ink */
/* the slots of each mask: each non-permanent brush in a slot holds one ref,
 * dropped when that mask buffer is rebuilt. The pen thread reads a slot only
 * inside ink_hz (a hazard: "I am reading mask c", c + 1), published before and
 * re-checked against ink_cur; the main thread rebuilds the other buffer only
 * once ink_hz does not name it, so a slot is never rewritten (nor its brush
 * freed) under the pen thread's feet. A stroke takes its own ref inside that
 * window and drops it at its end (bh_sweep frees on the main thread). */
static inkslot ink_slot[2][256];
static int ink_ns[2];                   /* slots used per buffer (slot 0 = none) */
static int ink_cur;                     /* the pen thread's mask (atomic) */
static int ink_hz;                      /* the pen thread's hazard: mask + 1 (0: none; atomic) */
static int ink_areas;                   /* sway sent areas: the mask decides (else ink_window) */

/* a rect's brush: sway's (the app set it), else its profile's brush=/eraser=,
 * else the default; per end. *tid / *rid: sway's brush file of that end when
 * sway's end won ("" none) */
static inkbrush ink_brush_for(int sbr, const char *app, const char **tid, const char **rid)
{
    static const swbrush none;
    const swbrush *sw = sbr > 0 && sbr <= INK_MAXB ? &ink_b[sbr] : &none;
    inkbrush d = BRUSH_DEFAULT, p = profile_for(app)->br, s = sw->v;
    *tid = *rid = "";
    if (p.tip.kind) d.tip = p.tip;
    if (p.rub.kind) d.rub = p.rub;
    if (s.tip.kind) { d.tip = s.tip; *tid = sw->tip; }
    if (s.rub.kind) { d.rub = s.rub; *rid = sw->rub; }
    return d;
}

/* one end -> its slot entry (main thread): the brush file if any and good
 * (its ink kind none: no preview), else the disc of the line's kind */
static inkend ink_end(brush v, const char *id)
{
    if (id[0]) {
        inkbh *h = bh_get(id);
        if (h) return epdb_kind(h->b) == NB_NONE ? (inkend){ 0 } : (inkend){ h, v.wmin, v.wmax, v.exp };
    }
    if (v.kind == BR_PEN || v.kind == BR_ERASE) return (inkend){ &ink_disc_h[v.kind == BR_ERASE], v.wmin, v.wmax, v.exp };
    return (inkend){ 0 };
}
static int inkend_eq(inkend a, inkend b) { return a.h == b.h && a.dmin == b.dmin && a.dmax == b.dmax && a.exp == b.exp && a.gate == b.gate; }

static void ink_rebuild(void)
{
    if (!ink_areas) return;
    ink_init();
    int nb = !__atomic_load_n(&ink_cur, __ATOMIC_SEQ_CST);
    while (__atomic_load_n(&ink_hz, __ATOMIC_SEQ_CST) == nb + 1) sched_yield();   /* (ns: one slot read) */
    uint8_t *m = ink_mask[nb];
    inkslot *sl = ink_slot[nb];
    for (int k = 1; k < ink_ns[nb]; k++) { bh_drop(sl[k].tip.h); bh_drop(sl[k].rub.h); }
    memset(m, 0, sizeof ink_mask[0]);
    int cells = 0, ns = 1;
    for (int i = 0; i < ink_n; i++) {
        if (!ink_r[i].sdk && !profile_for(ink_r[i].app)->ink) continue;
        const char *tid, *rid;
        inkbrush br = ink_brush_for(ink_r[i].br, ink_r[i].app, &tid, &rid);
        inkslot e = { ink_end(br.tip, tid), ink_end(br.rub, rid) };
        if (ink_r[i].sdk) e.tip.gate = e.rub.gate = INK_START_SDK;   /* (an app's canvas: no buttons) */
        int k = 1;
        while (k < ns && !(inkend_eq(sl[k].tip, e.tip) && inkend_eq(sl[k].rub, e.rub))) k++;
        if (k == ns) {
            if (ns == 256) k = 1;       /* (never: <= INK_MAXR rects, 255 slots) */
            else { sl[ns++] = e; bh_hold(e.tip.h); bh_hold(e.rub.h); }
        }
        box b = ink_r[i].b;
        vlog("  ink %s %d,%d %dx%d %s brush %d: %s %.1f-%.1f^%.2f eraser %s %.1f-%.1f^%.2f\n", ink_r[i].sdk ? "sdk" : "app",
             b.x0, b.y0, b.x1 - b.x0, b.y1 - b.y0, ink_r[i].app, k, sl[k].tip.h ? sl[k].tip.h->id : "none",
             sl[k].tip.dmin, sl[k].tip.dmax, sl[k].tip.exp, sl[k].rub.h ? sl[k].rub.h->id : "none",
             sl[k].rub.dmin, sl[k].rub.dmax, sl[k].rub.exp);
        for (int cy = (b.y0 + INK_C / 2) / INK_C; cy < INK_CH && cy * INK_C + INK_C / 2 < b.y1; cy++)
            for (int cx = (b.x0 + INK_C / 2) / INK_C; cx < INK_CW && cx * INK_C + INK_C / 2 < b.x1; cx++) {
                if (cy * INK_C + INK_C / 2 < b.y0 || cx * INK_C + INK_C / 2 < b.x0) continue;
                m[cy * INK_CW + cx] = (uint8_t)k;
                cells++;
            }
    }
    ink_ns[nb] = ns;
    __atomic_store_n(&ink_cur, nb, __ATOMIC_SEQ_CST);
    bh_sweep();
    vlog("ink areas: %d rects, %d cells, %d brushes\n", ink_n, cells, ns - 1);
}

/* may the pen ink at portrait (px,py)? With e: the brush there for that end
 * of the pen, with a ref the caller drops (bh_drop) */
static int ink_at(int px, int py, int rubber, inkend *e)
{
    if (e) *e = (inkend){ 0 };
    if (ink_mode == 0) return 0;
    if (!ink_areas) {                   /* X: the active window's app, the default disc */
        if (e && !rubber) *e = (inkend){ &ink_disc_h[0], BRUSH_DEFAULT.tip.wmin, BRUSH_DEFAULT.tip.wmax, BRUSH_DEFAULT.tip.exp };
        return ink_window;
    }
    if (px < 0 || py < 0 || px >= PH || py >= PW) return 0;
    size_t o = (size_t)(py / INK_C) * INK_CW + px / INK_C;
    if (!e) return ink_mask[__atomic_load_n(&ink_cur, __ATOMIC_ACQUIRE)][o] != 0;
    int c;
    do {
        c = __atomic_load_n(&ink_cur, __ATOMIC_SEQ_CST);
        __atomic_store_n(&ink_hz, c + 1, __ATOMIC_SEQ_CST);
    } while (__atomic_load_n(&ink_cur, __ATOMIC_SEQ_CST) != c);
    int k = ink_mask[c][o];
    if (k) {
        *e = rubber ? ink_slot[c][k].rub : ink_slot[c][k].tip;
        bh_hold(e->h);
    }
    __atomic_store_n(&ink_hz, 0, __ATOMIC_RELEASE);
    return k != 0;
}

/* an "ink ..." line from sway */
static void ink_line_cmd(const char *l)
{
    int x, y, w, h, s, nc = 0;
    if (!strcmp(l, "ink begin")) { ink_nn = 0; memset(ink_nb, 0, sizeof ink_nb); return; }
    if (!strcmp(l, "ink end")) {
        if (ink_nn < 0) return;         /* its begin was lost: keep the areas in force */
        memcpy(ink_r, ink_nr, ink_nn * sizeof ink_r[0]);
        memcpy(ink_b, ink_nb, sizeof ink_b);
        ink_n = ink_nn; ink_nn = -1; ink_areas = 1;
        ink_rebuild();                  /* (brush files are read here, main thread) */
        return;
    }
    if (ink_nn < 0) return;
    if (!strncmp(l, "ink brush ", 10)) {
        int k, tk, rk, n = 0;
        float t0, t1, te, r0, r1, re;
        if (sscanf(l, "ink brush %d %d %f %f %f %d %f %f %f%n", &k, &tk, &t0, &t1, &te, &rk, &r0, &r1, &re, &n) == 9 &&
            k > 0 && k <= INK_MAXB) {
            swbrush *sb = &ink_nb[k];
            *sb = (swbrush){ { { (uint8_t)(tk & 3), t0, t1, te }, { (uint8_t)(rk & 3), r0, r1, re } }, "", "" };
            /* v3: [tip=<id>] [rub=<id>] (anything else: ignored) */
            for (const char *t = l + n; *t; ) {
                while (*t == ' ') t++;
                size_t len = strcspn(t, " ");
                char *dst = !strncmp(t, "tip=", 4) ? sb->tip : !strncmp(t, "rub=", 4) ? sb->rub : NULL;
                if (dst && len == 20) {
                    memcpy(dst, t + 4, 16);
                    dst[16] = 0;
                    if (!brush_id_ok(dst)) { vlog("ink brush %d: bad id %.20s\n", k, t); dst[0] = 0; }
                }
                t += len;
            }
        }
        return;
    }
    if (ink_nn >= INK_MAXR) return;
    int br = 0;
    if (!strncmp(l, "ink r ", 6)) {
        if (sscanf(l, "ink r %d %d %d %d %d %d %n", &br, &s, &x, &y, &w, &h, &nc) != 6 || !nc) return;
        if (br < 0 || br > INK_MAXB) br = 0;
    } else if (sscanf(l, "ink %d %d %d %d %d %n", &s, &x, &y, &w, &h, &nc) != 5 || !nc) return;
    ink_nr[ink_nn].br = br;
    ink_nr[ink_nn].b = box_clip((box){ x, y, x + w, y + h }, (box){ 0, 0, PH, PW });
    ink_nr[ink_nn].sdk = s;
    snprintf(ink_nr[ink_nn].app, sizeof ink_nr[0].app, "%s", l + nc);
    ink_nn++;
}

static const profile *wl_profile_at(int x, int y, box *win)
{
    *win = (box){ 0, 0, PH, PW };
    for (int i = nshell - 1; i >= 0; i--)    /* the shell is above every app */
        if (x >= shellr[i].x0 && x < shellr[i].x1 && y >= shellr[i].y0 && y < shellr[i].y1) {
            *win = shellr[i];
            return &shellprof;
        }
    for (int i = nwlapp - 1; i >= 0; i--)
        if (x >= wlapp[i].b.x0 && x < wlapp[i].b.x1 && y >= wlapp[i].b.y0 && y < wlapp[i].b.y1) {
            *win = wlapp[i].b;
            return profile_for(wlapp[i].cls);
        }
    /* the bar (glassine-shell-wl): its own window, as on X */
    if (!box_empty(wl_area) && y < wl_area.y0) { *win = (box){ 0, 0, PH, wl_area.y0 }; return &shellprof; }
    return &defprof;
}

/* finger on the touchscreen (cyttsp5_mt): its reports go to the motion estimate
 * (epd-motion, one layer for "is the screen moving"; replaced the
 * TOUCH_MOVE gate, 37 px of travel before a burst counted as a gesture). The
 * "touch" profile applies to GESTURES only (epd-motion: the finger moved
 * MO_SLOP, a second finger, or the content shifted under a finger) while down
 * and MO_HOLD_MS after. A tap (button, KOReader page turn) is not a gesture:
 * its repaint follows the app's normal mode (touch=fast on taps made
 * every page turn a dithered DU). */
static int touch_fd = -1, touch_down;
static motion mo;
/* pointer parking: X moves the core pointer to every tap and leaves it there, so
 * the app sees a mouse resting on the tapped button -> hover tooltips/highlights
 * that stay until the next tap elsewhere. PARK_DELAY_MS after a finger
 * lifts (the tap's own release must reach the app first) the pointer is warped
 * onto a 1x1 InputOnly override-redirect window in the bottom-right corner,
 * raised above everything, so no app gets a hover. Pen and mouse hover unchanged. */
#define PARK_DELAY_MS 150
/* default off : dropdowns became unreliable to use by touch — the warp
 * reaches GTK while a combo's popup is opening/grabbing, and the selection tap
 * gets lost. Tooltips-after-taps return; FIFO park=on brings it back. */
static int park_on = 0;
static int64_t park_at;
static Window park_win;
static int touch_xmax = 1, touch_ymax = 1;
/* the first finger's position for epd-motion, in screen heights: touch x is
 * taken as the short side (1404/1872 of a height) — a rotated panel misjudges
 * by <= 4/3 */
static int touch_slot, tv_nx = -1, tv_ny = -1;
/* the touchscreen: "Glassine Touch" (glassine-touchd's clone; the raw device is grabbed by
 * it and silent) if it exists, else the raw "cyttsp5_mt". touch_recheck opens
 * it again when it went away or the clone appeared later. */
static int touch_is_clone;
static int64_t touch_checked;
static void vlog(const char *fmt, ...);
static void touch_open(void)
{
    int best = -1, clone = 0;
    DIR *d = opendir("/dev/input");
    struct dirent *e;
    while (d && (e = readdir(d)) && !clone) {
        if (strncmp(e->d_name, "event", 5)) continue;
        char path[300], name[128] = "";
        snprintf(path, sizeof path, "/dev/input/%s", e->d_name);
        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) continue;
        ioctl(fd, EVIOCGNAME(sizeof name), name);
        int is_clone = !strcmp(name, "Glassine Touch");
        if (is_clone || (best < 0 && strstr(name, "cyttsp5"))) {
            if (best >= 0) close(best);
            best = fd;
            clone = is_clone;
        } else {
            close(fd);
        }
    }
    if (d) closedir(d);
    if (best < 0) return;
    /* the same device again (the raw one under sway, where glassine-touchd is off and
     * this ran every 3 s): keep the open one — a reopen dropped touch_down
     * mid-gesture, the next press was never seen, so the speed froze at its
     * value then and a fling's lift was lost (the step never promoted, 10-02) */
    struct stat sa, sb;
    if (touch_fd >= 0 && !fstat(touch_fd, &sa) && !fstat(best, &sb) && sa.st_rdev == sb.st_rdev) {
        close(best);
        touch_is_clone = clone;
        return;
    }
    struct input_absinfo a;
    if (ioctl(best, EVIOCGABS(ABS_MT_POSITION_X), &a) == 0 && a.maximum > 0) touch_xmax = a.maximum;
    if (ioctl(best, EVIOCGABS(ABS_MT_POSITION_Y), &a) == 0 && a.maximum > 0) touch_ymax = a.maximum;
    if (touch_fd >= 0) close(touch_fd);
    touch_fd = best;
    touch_is_clone = clone;
    unsigned long keys[KEY_CNT / (8 * sizeof(long)) + 1] = { 0 };   /* a finger already down */
    touch_down = ioctl(best, EVIOCGKEY(sizeof keys), keys) >= 0 &&
                 (keys[BTN_TOUCH / (8 * sizeof(long))] >> (BTN_TOUCH % (8 * sizeof(long)))) & 1;
    /* slot 0's current position from the kernel (EVIOCGABS keeps no value for
     * MT axes: they live per slot). A coordinate is only sent when it changes,
     * so starting at -1 lost a constant axis for good: opened mid-stroke (the
     * 3 s recheck vs scroll-inject's straight strokes), motion stayed "still"
     * for the whole run */
    tv_nx = tv_ny = -1; touch_slot = 0;
    struct { __u32 code; __s32 v[16]; } ms;
    ms.code = ABS_MT_POSITION_X;
    if (ioctl(best, EVIOCGMTSLOTS(sizeof ms), &ms) >= 0) tv_nx = ms.v[0];
    ms.code = ABS_MT_POSITION_Y;
    if (ioctl(best, EVIOCGMTSLOTS(sizeof ms), &ms) >= 0) tv_ny = ms.v[0];
    if (mo.down) mo_up(&mo, now_ms());          /* a finger already down is seen at its next report */
}
static void touch_recheck(void)
{
    if ((touch_fd >= 0 && touch_is_clone) || now_ms() - touch_checked < 3000) return;
    touch_checked = now_ms();
    touch_open();
}
static int touch_read(void)
{
    struct input_event ev[64];
    ssize_t k;
    int any = 0;
    while (touch_fd >= 0 && (k = read(touch_fd, ev, sizeof ev)) > 0)
        for (int i = 0; i < (int)(k / sizeof *ev); i++) {
            struct input_event *e = &ev[i];
            any = 1;
            if (e->type == EV_SYN && e->code == SYN_REPORT && touch_down && tv_nx >= 0 && tv_ny >= 0) {
                double x = (double)tv_nx / touch_xmax * 0.75, y = (double)tv_ny / touch_ymax;
                if (!mo.down) mo_down(&mo, now_ms(), x, y); else mo_move(&mo, now_ms(), x, y);
            }
            if (e->type == EV_ABS && e->code == ABS_MT_SLOT) touch_slot = e->value;
            if (e->type == EV_ABS && touch_slot == 0 && e->code == ABS_MT_POSITION_X) tv_nx = e->value;
            if (e->type == EV_ABS && touch_slot == 0 && e->code == ABS_MT_POSITION_Y) tv_ny = e->value;
            if (e->type == EV_KEY && e->code == BTN_TOUCH) {
                vlog("touch %s\n", e->value ? "down" : "up");
                /* no position reset here: the frame's positions came BEFORE this
                 * key (kernel and scroll-inject order), and evdev never resends
                 * an unchanged coordinate (slot state persists across
                 * contacts), so tv_nx/tv_ny already mirror the new finger. The
                 * reset made mo_down wait until both axes had changed again:
                 * never for scroll-inject's straight strokes, motion stayed
                 * "still" in every run */
                if (e->value) touch_slot = 0;
                pol_tap(&pol, now_ms(), e->value, touch_xmax > 0 ? tv_nx * pol.W / touch_xmax : 0, touch_ymax > 0 ? tv_ny * pol.H / touch_ymax : 0);
                if (touch_down && !e->value) { park_at = now_ms() + PARK_DELAY_MS; mo_up(&mo, now_ms()); }
                touch_down = e->value;
            } else if (e->type == EV_ABS && e->code == ABS_MT_SLOT && e->value > 0 && touch_down) {
                mo_pinch(&mo, now_ms());                        /* 2nd finger: pinch */
            }
        }
    if (touch_fd >= 0 && k < 0 && errno != EAGAIN) {   /* gone: poll would spin */
        close(touch_fd);
        touch_fd = -1;
        touch_down = 0;
    }
    return any;
}
/* keyboards and mice under sway : under X every XInput event is
 * input (on_input); under sway epdd saw only the touchscreen, the pen and the
 * pointer's moves. A touchpad's two-finger scroll (wheel events, the cursor
 * still) and typing fell to quiet pacing (one read per quiet_ms, 5 s after the
 * last input) and their runs ended at idle_ms: a jerky scroll that "struggled
 * into motion" (ProtoArc XK01). Read-only, no grab, sway still gets
 * every event: any event here is input. act_recheck looks for new devices
 * every ACT_SCAN_MS (Bluetooth ones come and go). Touchscreens and pens have
 * their own readers (absolute + direct). */
#define ACT_MAX 8
#define ACT_SCAN_MS 3000
static int act_fd[ACT_MAX], act_n;
static dev_t act_dev[ACT_MAX];
static struct { dev_t dev; ino_t ino; } act_skip[32];  /* looked at, not wanted: never opened again (the inode:
                                                          * a node reused for a new device is a new one) */
static int act_nskip;
static int64_t act_checked = -100000;
#define BITS(n) ((n) / (8 * (int)sizeof(long)) + 1)
static int bit_set(const unsigned long *b, int n) { return (b[n / (8 * sizeof(long))] >> (n % (8 * sizeof(long)))) & 1; }
static int act_wanted(int fd, const char *name)
{
    unsigned long ev[BITS(EV_MAX)] = { 0 }, key[BITS(KEY_MAX)] = { 0 }, rel[BITS(REL_MAX)] = { 0 }, prop[BITS(INPUT_PROP_MAX)] = { 0 };
    if (!strcmp(name, "Glassine Touch") || strstr(name, "cyttsp5")) return 0;
    if (ioctl(fd, EVIOCGBIT(0, sizeof ev), ev) < 0) return 0;
    ioctl(fd, EVIOCGPROP(sizeof prop), prop);
    if (bit_set(prop, INPUT_PROP_DIRECT)) return 0;         /* touchscreens, pens */
    if (bit_set(ev, EV_KEY) && ioctl(fd, EVIOCGBIT(EV_KEY, sizeof key), key) >= 0 &&
        bit_set(key, KEY_A) && bit_set(key, KEY_Z)) return 1;  /* a keyboard (not gpio-keys' power key; the
                                                                 * XK01's has an ABS_VOLUME axis too) */
    if (bit_set(ev, EV_REL) && ioctl(fd, EVIOCGBIT(EV_REL, sizeof rel), rel) >= 0 &&
        (bit_set(rel, REL_X) || bit_set(rel, REL_WHEEL))) return 1;     /* a mouse (the XK01's pad is one) */
    return bit_set(ev, EV_ABS) && bit_set(prop, INPUT_PROP_POINTER);   /* a touchpad */
}
static void act_close(int i)
{
    close(act_fd[i]);
    act_fd[i] = act_fd[act_n - 1];
    act_dev[i] = act_dev[act_n - 1];
    act_n--;
}
static void act_recheck(void)
{
    if (now_ms() - act_checked < ACT_SCAN_MS) return;
    act_checked = now_ms();
    DIR *d = opendir("/dev/input");
    struct dirent *e;
    while (d && (e = readdir(d)) && act_n < ACT_MAX) {
        if (strncmp(e->d_name, "event", 5)) continue;
        char path[300], name[128] = "";
        snprintf(path, sizeof path, "/dev/input/%s", e->d_name);
        struct stat st;
        if (stat(path, &st)) continue;
        int known = 0;
        for (int i = 0; i < act_n; i++) known |= act_dev[i] == st.st_rdev;
        for (int i = 0; i < act_nskip; i++) known |= act_skip[i].dev == st.st_rdev && act_skip[i].ino == st.st_ino;
        if (known) continue;
        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) continue;
        ioctl(fd, EVIOCGNAME(sizeof name), name);
        if (!act_wanted(fd, name)) {
            close(fd);
            if (act_nskip < (int)(sizeof act_skip / sizeof *act_skip))
                act_skip[act_nskip].dev = st.st_rdev, act_skip[act_nskip++].ino = st.st_ino;
            continue;
        }
        act_fd[act_n] = fd;
        act_dev[act_n++] = st.st_rdev;
        vlog("input: %s (%s)\n", name, e->d_name);
    }
    if (d) closedir(d);
}
/* any event from the keyboards and mice (a scan: a stat per device, an open
 * only for one not seen before) */
static int act_read(void)
{
    int any = 0, wheel_seen = 0;
    for (int i = 0; i < act_n; i++) {
        struct input_event ev[64];
        ssize_t k;
        while ((k = read(act_fd[i], ev, sizeof ev)) > 0) {
            any = 1;
            /* a wheel (the touchpad's two-finger scroll is one): the scroll's finger down */
            for (size_t j = 0; j < (size_t)k / sizeof *ev; j++)
                if (ev[j].type == EV_REL && (ev[j].code == REL_WHEEL || ev[j].code == REL_HWHEEL ||
                                             ev[j].code == REL_WHEEL_HI_RES || ev[j].code == REL_HWHEEL_HI_RES)) {
                    if (!wheel_seen) vlog("wheel\n");
                    wheel_seen = 1;
                }
        }
        if (k < 0 && errno != EAGAIN) { vlog("input: device gone\n"); act_close(i--); }   /* (unplugged: poll would spin) */
    }
    if (wheel_seen) pol_wheel(&pol, now_ms());
    return any;
}
static void park(Display *dpy, Window root, int W, int H)
{
    if (!park_win) {
        XSetWindowAttributes a = { .override_redirect = True };
        park_win = XCreateWindow(dpy, root, W - 1, H - 1, 1, 1, 0, 0, InputOnly, CopyFromParent,
                                 CWOverrideRedirect, &a);
        XMapWindow(dpy, park_win);
    }
    XRaiseWindow(dpy, park_win);
    XWarpPointer(dpy, None, root, 0, 0, 0, 0, W - 1, H - 1);
    XFlush(dpy);
}

/* the finger's speed the policy sees (pol_finger: its lift starts the fling
 * decay): 0 unless the motion estimate says moving and the finger moved in the
 * last MO_FRESH_MS */
static double finger_speed(void)
{
    int64_t t = now_ms();
    return mo_state(&mo, t, NULL) == MO_MOVING && mo.down && t - mo.fmoved < MO_FRESH_MS ? mo.fv : 0;
}

/* epd-motion's content evidence for a burst: did its largest changed area shift
 * against what the panel shows (shadow)? A frame row is one portrait column
 * (index = portrait y reversed), so a vertical scroll is a shift along it.
 * 12 columns of the area; under buf_mu (shadow). Returns whether it measured. */
#define SHIFT_COLS 12
static unsigned long shift_n;           /* stats: measurements, their time */
static double shift_ms;
static int measure_shift(const frame *fr, int nfr, const boxlist *ch, int *shift, double *score)
{
    *shift = 0; *score = 0;
    if (!ch->n) return 0;
    box c = ch->b[0];
    for (int i = 1; i < ch->n; i++) if (box_area(ch->b[i]) > box_area(c)) c = ch->b[i];
    const frame *f = NULL;
    for (int i = 0; i < nfr && !f; i++) if (fr[i].g && box_inside(c, fr[i].b)) f = &fr[i];
    int n = c.y1 - c.y0, w = c.x1 - c.x0;
    if (!f || n < 64 || w < 1) return 0;
    /* 4 neighbouring columns averaged (mo_avg4): the panel buffer holds a run's
     * dithered black/white */
    static uint8_t ob[SHIFT_COLS][PW], nb[SHIFT_COLS][PW];
    const uint8_t *o[SHIFT_COLS], *nw[SHIFT_COLS];
    if (w < 4) return 0;
    int k = w / 4 < SHIFT_COLS ? w / 4 : SHIFT_COLS, i0 = f->b.y1 - c.y1;
    for (int r = 0; r < k; r++) {
        int px = c.x0 + (2 * r + 1) * (w - 3) / (2 * k);
        mo_avg4(nb[r], f->g + (size_t)(px - f->b.x0) * f->s + i0, f->s, n);
        mo_avg4(ob[r], shadow + (size_t)px * PW + f->bx0 + i0, PW, n);
        o[r] = ob[r]; nw[r] = nb[r];
    }
    double t0 = tsec();
    *shift = mo_find_shift(o, nw, k, n, n / 2, score);
    shift_ms += (tsec() - t0) * 1e3;
    shift_n++;
    return 1;
}

/* Pen preview drawing (epd-brush.c, brushes v3): dabs into shadow AND ebuf
 * (the controller now believes them there; the app's rendering is compared
 * to that), dithered by the engine's ordered table (C_DITHER: blue noise,
 * bayer; dizzy's ordered stand-in is blue) so every pixel is black or white
 * for the DU. The pen thread keeps its own copy of the table. */
static uint8_t pen_dith[64][64];
static int pen_dith_pat = -1;
static void pen_dither(void)
{
    int pat = dither_pattern == EPD_DITHER_DIZZY ? EPD_DITHER_BLUE : dither_pattern;
    if (pat != pen_dith_pat) epd_dither_table(pen_dith, pen_dith_pat = pat);
}
/* (the falloff,: ink_halo=on|off (ctl,
 * default off). On: a dab's soft edge (< 50 % coverage) goes to halo_plane
 * first and reaches the panel <= INK_HALO_MS later as its own DU (the core
 * goes out at once). Pen thread: halo_pend (panel px) since halo_t. */
#define INK_HALO_MS 12                  /* ~one 85 Hz frame */
static int ink_halo;                    /* atomic; halo_plane is set before it turns on */
static uint8_t *halo_plane;             /* PW x PH, allocated once (main) */
static epdb_box halo_pend;
static double halo_t;
static void ink_halo_set(int on)         /* (main) */
{
    if (on && !halo_plane && !(halo_plane = calloc((size_t)PW * PH, 1))) on = 0;
    __atomic_store_n(&ink_halo, on, __ATOMIC_RELEASE);
    fprintf(stderr, "glassine-epdd-desktop: ink_halo=%s\n", on ? "on" : "off");
}
/* pen-thread time in epdb_segment ("stats"; under buf_mu) */
static struct { unsigned long n, halo_n; double us, max_us; } pst;

static epdb_target pen_target(int halo)
{
    return (epdb_target){ shadow, ebuf, PW, PH, PW, halo ? halo_plane : NULL, &pen_dith[0][0], 64, 64 };
}
/* panel box -> portrait: panel (bx,by) = portrait (px = by, py = PW-1-bx) */
static box pbox(epdb_box w)
{
    if (w.x0 >= w.x1 || w.y0 >= w.y1) return (box){0};
    return box_clip((box){ w.y0, PW - w.x1, w.y1, PW - w.x0 }, (box){ 0, 0, PH, PW });
}
/* one segment of a stroke (buf_mu held); returns the portrait box written */
static box ink_seg(const inkend *e, epdb_stroke *st, const epdb_point *pt, int halo)
{
    epdb_target t = pen_target(halo);
    epdb_box hb = { 0, 0, 0, 0 };
    double t0 = tsec();
    epdb_box w = epdb_segment(e->h->b, e->dmin, e->dmax, e->exp, st, pt, &t, &hb);
    double us = (tsec() - t0) * 1e6;
    pst.n++; pst.us += us;
    if (us > pst.max_us) pst.max_us = us;
    if (w.x0 < w.x1 && w.y0 < w.y1) gt_mark((epd_ingest_rect){ w.x0, w.y0, w.x1 - w.x0, w.y1 - w.y0 });
    if (halo && hb.x0 < hb.x1 && hb.y0 < hb.y1) {
        if (halo_pend.x0 >= halo_pend.x1) { halo_pend = hb; halo_t = tsec(); }
        else {
            if (hb.x0 < halo_pend.x0) halo_pend.x0 = hb.x0;
            if (hb.y0 < halo_pend.y0) halo_pend.y0 = hb.y0;
            if (hb.x1 > halo_pend.x1) halo_pend.x1 = hb.x1;
            if (hb.y1 > halo_pend.y1) halo_pend.y1 = hb.y1;
        }
    }
    return pbox(w);
}

/* tilt raw -> degrees, -90..90. resolution: units per radian (hid-input);
 * without one the range is taken as +-60 degrees (assumption, unmeasured) */
static float tilt_deg(int v, int mn, int mx, int res)
{
    double c = mn < 0 ? 0 : (mn + mx) / 2.0;
    double d = res > 0 ? (v - c) / res * (180 / M_PI) : (v - c) / ((mx - mn) / 2.0) * 60;
    return (float)(d < -90 ? -90 : d > 90 ? 90 : d);
}

/* the stroke ends (lift, left the ink area, a button): its brush ref goes */
static int pen_end(struct pen *p)
{
    int had = p->br.h != NULL;
    bh_drop(p->br.h);
    p->br = (inkend){ 0 };
    p->drawing = 0;
    p->sx = -1;
    return had;
}

/* read all pen events; draw; returns the portrait box inked (empty: none);
 * *dropped: a stroke let go of its brush (the main loop sweeps) */
static box pen_read(struct pen *p, int *lost, int *dropped)
{
    box acc = {0};
    struct input_event ev[64];
    for (;;) {
        ssize_t k = read(p->fd, ev, sizeof ev);
        if (k < 0) { if (errno == ENODEV) *lost = 1; break; }
        if (k == 0) break;
        for (int i = 0; i < (int)(k / sizeof *ev); i++) {
            struct input_event *e = &ev[i];
            if (e->type == EV_ABS) {
                if (e->code == ABS_X) p->x = e->value;
                else if (e->code == ABS_Y) p->y = e->value;
                else if (e->code == ABS_PRESSURE) p->p = e->value;
                else if (e->code == ABS_TILT_X) p->tx = e->value;
                else if (e->code == ABS_TILT_Y) p->ty = e->value;
            } else if (e->type == EV_KEY) {
                if (e->code == BTN_TOUCH) p->touch = e->value;
                else if (e->code == BTN_TOOL_PEN) p->tool_pen = e->value;
                else if (e->code == BTN_TOOL_RUBBER) p->rubber = e->value;
                else if (e->code == BTN_STYLUS || e->code == BTN_STYLUS2)
                    p->btn = e->value ? p->btn | (1 << (e->code - BTN_STYLUS)) : p->btn & ~(1 << (e->code - BTN_STYLUS));
            } else if (e->type == EV_SYN && e->code == SYN_REPORT) {
                /* pen (x,y) normalized -> panel buffer: same as the X calibration
                 * matrix (portrait px = y, py = 1 - x) composed with ebuf's rotation */
                float fx = (float)(p->x - p->minx) * (PW - 1) / (p->maxx - p->minx ? p->maxx - p->minx : 1);
                float fy = (float)(p->y - p->miny) * (PH - 1) / (p->maxy - p->miny ? p->maxy - p->miny : 1);
                if (p->flip) { fx = PW - 1 - fx; fy = PH - 1 - fy; }
                int bx = (int)fx, by = (int)fy;
                /* panel (bx,by) = portrait (px = by, py = PW-1-bx) */
                /* the brush is the stroke start's (its app's, for that end of the pen) */
                inkend ib;
                int down = p->touch && !p->btn && ink_at(by, PW - 1 - bx, p->rubber, p->sx < 0 ? &ib : NULL);
                if (!down) { *dropped |= pen_end(p); continue; }
                /* the report in panel buffer px; tilt in the same frame (the
                 * pen's x/y axes are the buffer's; flip turns both) */
                epdb_point pt = { fx, fy, 1, 0, 0, 0, e->input_event_sec + e->input_event_usec / 1e6 };
                if (p->maxp > 0) pt.pressure = (float)p->p / p->maxp;
                if (pt.pressure < 0) pt.pressure = 0;
                if (pt.pressure > 1) pt.pressure = 1;
                if (p->has_tilt) {
                    pt.tilt_x = tilt_deg(p->tx, p->tmin[0], p->tmax[0], p->tres[0]);
                    pt.tilt_y = tilt_deg(p->ty, p->tmin[1], p->tmax[1], p->tres[1]);
                    if (p->flip) { pt.tilt_x = -pt.tilt_x; pt.tilt_y = -pt.tilt_y; }
                    pt.has_tilt = 1;
                }
                if (p->sx < 0) { p->sx = bx; p->sy = by; p->lx = bx; p->ly = by; p->br = ib; p->sp = pt; }
                if (!p->br.h) continue;                  /* a brush without preview */
                if (!p->drawing && abs(bx - p->sx) + abs(by - p->sy) < (p->br.gate ? p->br.gate : INK_START_PX)) continue;
                int halo = __atomic_load_n(&ink_halo, __ATOMIC_ACQUIRE) && halo_plane;
                if (!p->drawing) {
                    pen_dither();
                    memset(&p->st, 0, sizeof p->st);
                    epdb_stroke_begin(&p->st, &p->sp, (uint32_t)(tsec() * 1e6) | 1);
                }
                pthread_mutex_lock(&buf_mu);
                acc = box_union(acc, ink_seg(&p->br, &p->st, &pt, halo));
                pthread_mutex_unlock(&buf_mu);
                p->drawing = 1; p->lx = bx; p->ly = by;
                p->inked_ms = now_ms();
            }
        }
    }
    return acc;
}

/* the pending halo out as its own DU (pen thread) */
static box halo_flush(void)
{
    box pb = {0};
    if (halo_pend.x0 >= halo_pend.x1 || !halo_plane) return pb;
    pthread_mutex_lock(&buf_mu);
    epdb_target t = pen_target(1);
    epdb_box w = epdb_halo_flush(&t, halo_pend);
    if (w.x0 < w.x1 && w.y0 < w.y1) gt_mark((epd_ingest_rect){ w.x0, w.y0, w.x1 - w.x0, w.y1 - w.y0 });
    pst.halo_n++;
    pthread_mutex_unlock(&buf_mu);
    halo_pend = (epdb_box){ 0, 0, 0, 0 };
    return pbox(w);
}

/* ---- pen thread ---------------------------------------------------------------
 * Pen events are read, drawn and submitted here, never behind the main loop (an
 * X snapshot, a burst's commit, profile lookups). The main loop learns what was
 * inked afterwards (ink_new + a byte on pen_pipe) for the policy's cleanup. */
static volatile int pen_ok = 1;            /* main: the policy is awake */
static volatile int pen_inking;            /* drawing, or the hover hold (INK_HOVER_MS) */
static volatile int64_t pen_input_ms;      /* last pen activity (touch / hover) */
static boxlist ink_new;                    /* inked since the main loop looked; buf_mu */
static int pen_pipe[2] = { -1, -1 };

static void pen_notify(void)
{
    char c = 1;
    if (write(pen_pipe[1], &c, 1) < 0) { /* full: the main loop is awake anyway */ }
}

/* "penbench": a synthetic thick diagonal stroke through the pen thread's path
 * (draw, submit, report), one piece every 2.8 ms (the Wacom's ~360 reports/s),
 * 4 px apart (a fast stroke) — repeatable load for the "stats" counters. The
 * preview is cleaned up like real ink. */
static void *pen_bench(void *arg)
{
    (void)arg;
    struct sched_param sp = { .sched_priority = 40 };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    int x0 = 200, y0 = 200, x1 = 1600, y1 = 1200;
    int n = (int)(hypot(x1 - x0, y1 - y0) / 4);
    /* the disc brush, 6 px (the old r 3), through the same renderer path */
    inkend disc = { &ink_disc_h[0], 6, 6, 1 };
    epdb_stroke st = {0};
    epdb_point pt = { x0, y0, 1, 0, 0, 0, tsec() };
    pen_dither();
    epdb_stroke_begin(&st, &pt, 0x9e3779b9u);
    pen_inking = 1;
    for (int i = 1; i <= n; i++) {
        pt.x = x0 + (float)(x1 - x0) * i / n; pt.y = y0 + (float)(y1 - y0) * i / n; pt.t = tsec();
        pthread_mutex_lock(&buf_mu);
        box b = ink_seg(&disc, &st, &pt, 0);
        send(b, WF_DU, M_PART, 0);
        list_add(&ink_new, b);
        pthread_mutex_unlock(&buf_mu);
        pen_notify();
        usleep(2800);
    }
    pen_inking = 0;
    pen_notify();
    fprintf(stderr, "glassine-epdd-desktop: penbench: %d pieces\n", n);
    return NULL;
}

static void *pen_thread(void *arg)
{
    (void)arg;
    /* below the stream thread (50): ink must not delay a frame, but beats
     * everything else */
    struct sched_param sp = { .sched_priority = 40 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) && verbose)
        fprintf(stderr, "glassine-epdd-desktop: no SCHED_FIFO for the pen thread\n");
    pen_scan();
    int64_t last_scan = now_ms();
    int was = 0;
    for (;;) {
        struct pollfd pf[MAXPEN];
        for (int i = 0; i < npens; i++) pf[i] = (struct pollfd){ pens[i].fd, POLLIN, 0 };
        int timeout = 5000;
        for (int i = 0; i < npens; i++)         /* a still hovering pen: the hold ends on its own */
            if (!pens[i].drawing && (pens[i].tool_pen || pens[i].touch)) {
                int64_t dt = pens[i].inked_ms + INK_HOVER_MS - now_ms();
                if (dt >= 0 && dt < timeout) timeout = (int)dt + 1;
            }
        if (halo_pend.x0 < halo_pend.x1) {      /* the halo's deadline */
            int dt = (int)ceil((halo_t - tsec()) * 1e3) + INK_HALO_MS;
            if (dt < timeout) timeout = dt < 0 ? 0 : dt;
        }
        poll(pf, npens, timeout);
        int inking = 0, inked = 0, lost = 0, dropped = 0, drawing = 0;
        for (int i = 0; i < npens && !lost; i++) {
            struct pen *p = &pens[i];
            box b = pen_read(p, &lost, &dropped);
            if (lost) break;
            if (!box_empty(b) && pen_ok) {
                pthread_mutex_lock(&buf_mu);
                send(b, WF_DU, M_PART, 0);
                list_add(&ink_new, b);
                pthread_mutex_unlock(&buf_mu);
                inked = 1;
            }
            drawing |= p->drawing;
            inking |= p->drawing || ((p->tool_pen || p->touch) && now_ms() - p->inked_ms < INK_HOVER_MS);
            /* hovering too: the link (and rails) come up before the tip lands, so
             * the stroke's first ink doesn't wait the ~75 ms link wake-up */
            if ((p->touch || p->tool_pen) && now_ms() - pen_input_ms >= 100) {   /* ~360 reports/s: not each */
                pen_input_ms = now_ms();
                if (eng && eng_keepalive_ms > 0) epd_stream_keepalive(eng, eng_keepalive_ms);
            }
        }
        /* the halo after the core, once a frame old, or at the lift */
        if (halo_pend.x0 < halo_pend.x1 && (lost || !drawing || tsec() - halo_t >= INK_HALO_MS / 1e3)) {
            box hb = halo_flush();
            if (!box_empty(hb) && pen_ok) {
                pthread_mutex_lock(&buf_mu);
                send(hb, WF_DU, M_PART, 0);
                list_add(&ink_new, hb);
                pthread_mutex_unlock(&buf_mu);
                inked = 1;
            }
        }
        /* pick up (un)plugged pen devices; never mid-stroke (state is reset) */
        if (lost || (!inking && now_ms() - last_scan > 5000)) {
            pen_scan();
            last_scan = now_ms();
            inking = 0;
        }
        pen_inking = inking;
        if (inked || dropped || inking != was) pen_notify();
        was = inking;
    }
    return NULL;
}

/* ---- sleep banner (glassine-powerd: "sleep" before suspend, "wake" after) ------
 * The panel keeps its image without power, so a suspended device looks awake:
 * a black bar with "SLEEPING" at the bottom says it is not (epd-glyphs.h). */
static box sleep_banner(int W, int H)
{
    const char *text = "SLEEPING";
    box bar = { 0, H - BAR_H, W, H };
    /* into shadow too: it is what the panel shows now (and the engine reads
     * shadow); "wake" re-reads the bar with all=1, which puts the screen back */
    for (int py = bar.y0; py < bar.y1; py++)
        for (int px = 0; px < W; px++) {
            size_t o = (size_t)px * PW + (PW - 1 - py);
            uint8_t g = epd_bar_ink(text, (int)strlen(text), 6, W, BAR_H, px, py - bar.y0) ? 255 : 0;
            shadow[o] = g;
            if (ebuf) ebuf[o] = 0xFF000000u | g << 16 | g << 8 | g;
        }
    gt_mark((epd_ingest_rect){ PW - bar.y1, bar.x0, bar.y1 - bar.y0, bar.x1 - bar.x0 });   /* panel cols PW-y1.., rows x0.. */
    return bar;
}

/* ---- sleep pill (design round 8): glassine-powerd renders it with glassine-sleep-pill
 * (the shell's theme, turned with the screen) before it sends "sleep"; drawn
 * over the page instead of the bar. PAM GRAYSCALE_ALPHA, premultiplied, with
 * its place as "# at X Y" (logical px). Older than PILL_MAX_AGE s: stale (the
 * renderer failed this time), the bar instead. */
#define PILL_PATH "/run/glassine-sleep-pill.pam"
#define PILL_MAX_AGE 60
static uint8_t *pill_px;                /* grey, alpha pairs */
static box pill_box;

static void pill_load(int W, int H)
{
    free(pill_px); pill_px = NULL; pill_box = (box){0};
    struct stat st;
    if (stat(PILL_PATH, &st) || time(NULL) - st.st_mtime > PILL_MAX_AGE) return;
    FILE *f = fopen(PILL_PATH, "rb");
    if (!f) return;
    char l[128];
    int w = 0, h = 0, x = -1, y = -1, depth = 0, ok = 0;
    if (fgets(l, sizeof l, f) && !strncmp(l, "P7", 2))
        while (fgets(l, sizeof l, f)) {
            if (!strncmp(l, "ENDHDR", 6)) { ok = 1; break; }
            sscanf(l, "WIDTH %d", &w); sscanf(l, "HEIGHT %d", &h); sscanf(l, "DEPTH %d", &depth);
            sscanf(l, "# at %d %d", &x, &y);
        }
    if (ok && depth == 2 && w > 0 && h > 0 && x >= 0 && y >= 0 && x + w <= W && y + h <= H &&
        (pill_px = malloc((size_t)w * h * 2)) && fread(pill_px, 2, (size_t)w * h, f) == (size_t)w * h)
        pill_box = (box){ x, y, x + w, y + h };
    else { free(pill_px); pill_px = NULL; fprintf(stderr, "glassine-epdd-desktop: %s unreadable: the bar instead\n", PILL_PATH); }
    fclose(f);
}

/* over the page in shadow (and ebuf), like sleep_banner */
static void pill_draw(void)
{
    box b = pill_box;
    int w = b.x1 - b.x0;
    for (int py = b.y0; py < b.y1; py++)
        for (int px = b.x0; px < b.x1; px++) {
            const uint8_t *s = pill_px + ((size_t)(py - b.y0) * w + (px - b.x0)) * 2;
            size_t o = (size_t)px * PW + (PW - 1 - py);
            uint8_t g = (uint8_t)(s[0] + (255 - s[1]) * shadow[o] / 255);
            shadow[o] = g;
            if (ebuf) ebuf[o] = 0xFF000000u | g << 16 | g << 8 | g;
        }
    gt_mark((epd_ingest_rect){ PW - b.y1, b.x0, b.y1 - b.y0, b.x1 - b.x0 });
}

static void on_quit(int sig) { (void)sig; quit_sig = 1; }

/* engine: wait for the running updates, rails down, keep the levels */
static void eng_close(void)
{
    if (!eng) return;
    epd_stream_save_levels(eng, ENG_LEVELS);
    epd_stream_close(eng);
    eng = NULL;
}

static int xerr(Display *d, XErrorEvent *e) { (void)d; (void)e; return 0; }

/* ---- top-level windows (root children) for pol_window ------------------------
 * Their last known area: an UnmapNotify carries none, and the window may be
 * destroyed before we could ask. InputOnly windows (xfwm4's helpers, our own
 * park window) show nothing and are left out. */
#define MAXTOP 256
static struct { Window w; box b; int mapped, popup; } tops[MAXTOP];
static int ntops;

static int top_find(Window w)
{
    for (int i = 0; i < ntops; i++) if (tops[i].w == w) return i;
    return -1;
}

static int top_get(Display *dpy, Window w)
{
    int i = top_find(w);
    if (i >= 0) return i;
    XWindowAttributes wa;
    if (!XGetWindowAttributes(dpy, w, &wa) || wa.class == InputOnly) return -1;
    if (ntops == MAXTOP) {                      /* forget an unmapped one */
        for (i = 0; i < ntops && tops[i].mapped; i++) ;
        if (i == ntops) return -1;
        tops[i] = tops[--ntops];
    }
    int bw = wa.border_width;
    tops[ntops].w = w;
    tops[ntops].b = (box){ wa.x, wa.y, wa.x + wa.width + 2 * bw, wa.y + wa.height + 2 * bw };
    tops[ntops].mapped = wa.map_state != IsUnmapped;
    tops[ntops].popup = wa.override_redirect;
    return ntops++;
}

static int settling;                    /* see settle_hold */
static int64_t settle_t0, settle_dmg;

/* a dialog (WM_TRANSIENT_FOR, on the window or -- xfwm4 frames -- a child):
 * not tiled, nothing to wait for; its first paint stays fast */
static int is_transient(Display *dpy, Window w)
{
    Window tf, r, par, *ch = NULL;
    unsigned n = 0;
    if (XGetTransientForHint(dpy, w, &tf)) return 1;
    if (!XQueryTree(dpy, w, &r, &par, &ch, &n)) return 0;
    int t = 0;
    for (unsigned i = 0; i < n && !t; i++) t = XGetTransientForHint(dpy, ch[i], &tf);
    if (ch) XFree(ch);
    return t;
}

/* a root child's event: its area goes to the policy when what is on screen changes */
static void top_event(Display *dpy, const XEvent *ev)
{
    int64_t t = now_ms();
    if (ev->type == DestroyNotify) {
        int i = top_find(ev->xdestroywindow.window);
        if (i >= 0) tops[i] = tops[--ntops];
    } else if (ev->type == MapNotify) {
        int i = top_get(dpy, ev->xmap.window);
        if (i < 0) return;
        tops[i].mapped = 1;
        /* a popup/menu appearing: no flash (it would blink every menu ~0.6 s
         * after it opened) but a fast first paint; where it disappears
         * again, one GC16 partial */
        tops[i].popup = ev->xmap.override_redirect;
        if (!tops[i].popup) {
            pol_window(&pol, tops[i].b, t);
            /* a new window: xfwm4 places it, glassine-winfit tiles it (and resizes
             * the others), its app draws -- reads wait until that is over, as
             * after a workspace switch: one update instead of each half state */
            if (!pen_inking && !is_transient(dpy, ev->xmap.window)) {
                settling = 1; settle_t0 = settle_dmg = t;
                vlog("window mapped: settling\n");
            }
        } else pol_popup(&pol, tops[i].b, t);
    } else if (ev->type == UnmapNotify) {
        int i = top_find(ev->xunmap.window);
        if (i < 0 || !tops[i].mapped) return;
        tops[i].mapped = 0;
        /* a popup disappearing: one GC16 partial of what it uncovered */
        if (tops[i].popup) pol_popup_gone(&pol, tops[i].b, t);
        else pol_window(&pol, tops[i].b, t);
    } else if (ev->type == ConfigureNotify) {
        const XConfigureEvent *c = &ev->xconfigure;
        int i = top_get(dpy, c->window);
        if (i < 0) return;
        box nb = { c->x, c->y, c->x + c->width + 2 * c->border_width, c->y + c->height + 2 * c->border_width };
        box ob = tops[i].b;
        tops[i].b = nb;
        if (tops[i].mapped && memcmp(&ob, &nb, sizeof nb)) {
            pol_window(&pol, ob, t);
            pol_window(&pol, nb, t);
        }
    }
}

static void vlog(const char *fmt, ...)
{
    if (!verbose) return;
    fprintf(stderr, "%6lld ", (long long)(now_ms() % 1000000));    /* ms, for latency reading */
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
}

/* carry out the policy's decisions, in order */
static void run_actions(Display *dpy, Window root, const actions *o)
{
    for (int i = 0; i < o->n; i++) {
        const action *a = &o->a[i];
        switch (a->kind) {
        case A_SEND:     pthread_mutex_lock(&buf_mu); send(a->b, a->wf, a->mode, a->n); pthread_mutex_unlock(&buf_mu); break;
        case A_GRAB:     grab(dpy, root, a->b, 0, NULL); break;
        case A_GRAB_ALL: { boxlist ignore = {0}; grab(dpy, root, a->b, 1, &ignore); break; }
        case A_CUT:      pthread_mutex_lock(&buf_mu); set_cut(a->n); pthread_mutex_unlock(&buf_mu); break;
        case A_BANNER:   boost_set(0);           /* suspend follows: no clocks held up */
                         pthread_mutex_lock(&buf_mu);
                         if (pill_px) pill_draw(); else sleep_banner(pol.W, pol.H);
                         pthread_mutex_unlock(&buf_mu); break;
        }
    }
}

/* ---- impostors ----------------------------------------------------------------
 * A client that knows what is about to appear hands the pixels over before X has
 * them: "impostor X Y W H FILE" on the FIFO (glassine-shell's popups, whose
 * last rendering it keeps; ~250 ms tap -> ink the X way). FILE: raw a8r8g8b8,
 * W*H, portrait screen rect X,Y,W,H; alpha 0 = not part of the (shaped) window,
 * the panel keeps what it shows there. Only under /run/user/ (the FIFO is
 * world-writable). The pixels go to the panel at once as a popup's first paint
 * (pol_popup: dithered DU, the GC16 of the real greys right behind — its
 * re-read comes from the impostor, not from X, which has no popup yet). They
 * are then what the panel shows: X's real window arrives as ordinary damage and
 * only pixels that differ get painted. What X never confirms (closed
 * meanwhile, the client died) is re-read IMP_CHECK_MS later, like damage.
 * Same grey as the compositor: luma, then the gamma LUT. */
#define IMP_CHECK_MS 800
static boxlist imp_check;
static int64_t imp_check_at;
/* converted impostors (panel-order grey + shape mask) by file: a client
 * rewrites its file only when the pixels change, so opening the same popup
 * again skips the read and the conversion (~27 of ~47 ms) */
#define IMP_CACHE 4
static struct { char path[100]; struct timespec mt; int w, h; uint8_t *g, *m; int64_t used; } imc[IMP_CACHE];

/* the impostor's panel-order grey (into g) and mask, from the cache or the file; -1 on failure */
static int imp_load(const char *path, int w, int h, uint8_t *g, const uint8_t **mask, int64_t *t_file, int *cached)
{
    size_t n = (size_t)w * h;
    struct stat sb;
    if (stat(path, &sb) || sb.st_size != (off_t)(n * 4)) return -1;
    int k = -1, lru = 0;
    for (int i = 0; i < IMP_CACHE; i++) {
        if (!strcmp(imc[i].path, path)) k = i;
        if (imc[i].used < imc[lru].used) lru = i;
    }
    if (k >= 0 && imc[k].g && imc[k].w == w && imc[k].h == h &&
        imc[k].mt.tv_sec == sb.st_mtim.tv_sec && imc[k].mt.tv_nsec == sb.st_mtim.tv_nsec) {
        memcpy(g, imc[k].g, n);
        *mask = imc[k].m;
        *cached = 1;
        imc[k].used = now_ms();
        return 0;
    }
    if (k < 0) k = lru;
    free(imc[k].g); free(imc[k].m);
    memset(&imc[k], 0, sizeof imc[k]);
    int64_t t0 = now_ms();
    uint32_t *px = malloc(n * 4);
    FILE *f = px ? fopen(path, "rb") : NULL;
    int ok = f && fread(px, 4, n, f) == n;
    if (f) fclose(f);
    *t_file = now_ms() - t0;
    /* row by row, then turned in tiles (a per-pixel column walk of the
     * source took ~150 ms for the quick settings card, a cache miss a pixel);
     * alpha 0 as a mask, turned the same way */
    uint8_t *pg = ok ? malloc(n) : NULL, *pm = pg ? malloc(n) : NULL, *m = pm ? malloc(n) : NULL;
    if (!m) { free(pg); free(pm); free(px); return -1; }
    for (int r = 0; r < h; r++) {
        const uint32_t *row = px + (size_t)r * w;
        comp_grey_row(lut, row, pg + (size_t)r * w, w);
        for (int i = 0; i < w; i++) pm[(size_t)r * w + i] = !(row[i] >> 24);
    }
    free(px);
    comp_rotate(pg, g, w, h);
    comp_rotate(pm, m, w, h);
    free(pm);
    memcpy(pg, g, n);                           /* the cache's copy */
    snprintf(imc[k].path, sizeof imc[k].path, "%s", path);
    imc[k].mt = sb.st_mtim; imc[k].w = w; imc[k].h = h;
    imc[k].g = pg; imc[k].m = m;
    imc[k].used = now_ms();
    *mask = m;
    *cached = 0;
    return 0;
}

/* the dead band's true frames, here (not in the dispatch: the pen, a settle
 * wait): what in them differs from the panel goes to the policy, which cleans
 * it at the cleanup's usual time. A run's own GC16 may cover it all: then
 * nothing extra is sent. Asleep or paused nothing is read: wake and
 * resume compare the whole frame */
static void true_frames(Display *dpy, Window root, boxlist *pend, int64_t *first_dmg, int64_t t)
{
    boxlist ch = {0};
    if (pol.state != S_PAUSED && pol.state != S_ASLEEP) {
        frame fr[MAXBOX];
        for (int i = 0; i < wl_true.n; i++) fr[i] = snap(dpy, root, wl_true.b[i], src_fb);
        pthread_mutex_lock(&buf_mu);
        for (int i = 0; i < wl_true.n; i++) if (fr[i].g) scan_job(&fr[i], 0, &ch);
        pthread_mutex_unlock(&buf_mu);
        for (int i = 0; i < wl_true.n; i++) fbuf_put(fr[i].g);
        true_changed += ch.n > 0;
        if (verbose) fprintf(stderr, "%6lld true frame: %d rects -> %d changed\n", (long long)(t % 1000000), wl_true.n, ch.n);
        if (!pol_true(&pol, &ch))
            for (int i = 0; i < ch.n; i++) {
                if (!pend->n) *first_dmg = t;
                list_add(pend, ch.b[i]);
            }
    }
    wl_true.n = 0;
}

/* Pixels that don't come from X (an impostor, a workspace snapshot) to the
 * panel, as a burst of their own: popup = a popup's paint of the whole frame
 * (no scan: a partial update moves only pixels that change anyway), else the
 * changed pixels scanned and sized like any burst. The policy's re-reads
 * (A_GRAB) take the frame's pixels: X doesn't have them yet. The frame's area
 * is re-read from X IMP_CHECK_MS later, like damage (whatever X never
 * confirms). Returns when the commit was done (timing). */
static int64_t paint_frame(Display *dpy, Window root, const frame *fr, int popup, actions *out)
{
    burst_in in = { .now = now_ms(), .pwin = fr->b, .pm = PM_AUTO };
    if (popup) {
        list_add(&in.ch, fr->b);
        pol_popup(&pol, fr->b, in.now);
    } else {
        pthread_mutex_lock(&buf_mu);
        scan(fr, 0, &in.ch);
        pthread_mutex_unlock(&buf_mu);
    }
    out->n = 0;
    int how = pol_burst(&pol, &in, out);
    dz_prep(fr, 1, how);
    pthread_mutex_lock(&buf_mu);
    commit(fr, 0, how);
    pthread_mutex_unlock(&buf_mu);
    dz_prep_drop();
    int64_t t_commit = now_ms();
    for (int i = 0; i < out->n; i++) {
        const action *a = &out->a[i];
        if (a->kind == A_GRAB || a->kind == A_GRAB_ALL) {     /* the real greys: ours */
            pthread_mutex_lock(&buf_mu);
            commit(fr, a->kind == A_GRAB_ALL, C_ALL);
            pthread_mutex_unlock(&buf_mu);
        } else {
            actions one = { .n = 1 };
            one.a[0] = *a;
            run_actions(dpy, root, &one);
        }
    }
    out->n = 0;
    if (!imp_check.n) imp_check_at = now_ms() + IMP_CHECK_MS;
    list_add(&imp_check, fr->b);
    return t_commit;
}

/* the mouse pointer's own paint (see ptr_cb): where it was and where it is
 * now, read (with the stamp), scanned and painted as pol_pointer says, like
 * paint_frame. Only the newest place: a 125 Hz mouse is folded to what the
 * engine takes (the area free, PTR_MIN_MS apart). Returns ms until it wants
 * to look again (-1: nothing waits; the pen's end wakes the loop itself) */
static int ptr_step(int64_t now)
{
    box scr = { 0, 0, pol.W, pol.H };
    box nb = ptr_vis ? box_clip(ptr.b, scr) : (box){0};
    int same_box = !memcmp(&nb, &ptr_drawn.b, sizeof nb);
    if (ptr_vis == ptr_drawn.on && (!ptr_vis || (same_box && ptr.gen == ptr_drawn.gen))) return -1;
    if (pol.state == S_PAUSED || pol.state == S_ASLEEP || pen_inking) return -1;
    if (ptr_vis && ptr_drawn.on && same_box && now - ptr_drawn.t_img < PTR_IMG_MS)
        return (int)(PTR_IMG_MS - (now - ptr_drawn.t_img));     /* (an animated cursor: rate-limited) */
    if (now - ptr_drawn.t < PTR_MIN_MS) return (int)(PTR_MIN_MS - (now - ptr_drawn.t));
    boxlist u = {0};
    if (!box_empty(ptr_drawn.b)) list_add(&u, ptr_drawn.b);
    if (!box_empty(nb)) list_add(&u, nb);           /* (merged with the old place if near) */
    for (int i = 0; i < u.n; i++) {
        int w = eng_wait_box(u.b[i]);
        if (w > 0) return w;                        /* the engine is still on it: later, at the newest place */
    }
    frame fr[2];
    boxlist ch = {0};
    for (int i = 0; i < u.n; i++) fr[i] = snap(NULL, None, u.b[i], 0);
    pthread_mutex_lock(&buf_mu);
    for (int i = 0; i < u.n; i++) if (fr[i].g) scan(&fr[i], 0, &ch);
    pthread_mutex_unlock(&buf_mu);
    if (ch.n) {
        actions out = {0};
        int how = pol_pointer(&pol, &ch, now, &out);
        cm_motion = 1;                              /* (it moves) */
        dz_prep(fr, u.n, how);
        pthread_mutex_lock(&buf_mu);
        for (int i = 0; i < u.n; i++) if (fr[i].g) commit(&fr[i], 0, how);
        cm_motion = 0;
        pthread_mutex_unlock(&buf_mu);
        dz_prep_drop();
        run_actions(NULL, None, &out);
        ptr_paints++;
    }
    for (int i = 0; i < u.n; i++) fbuf_put(fr[i].g);
    vlog("pointer %s %d,%d-%d,%d: %d changed, %lld ms\n", ptr_vis ? "at" : "gone", nb.x0, nb.y0, nb.x1, nb.y1, ch.n,
         (long long)(now_ms() - now));
    if (ptr.gen != ptr_drawn.gen || ptr_vis != ptr_drawn.on) ptr_drawn.t_img = now;
    ptr_drawn.on = ptr_vis; ptr_drawn.b = nb; ptr_drawn.gen = ptr.gen; ptr_drawn.t = now;
    return -1;
}

static void impostor(Display *dpy, Window root, const char *cmd, actions *out)
{
    int x, y, w, h;
    char path[100];
    if (sscanf(cmd, "impostor %d %d %d %d %99s", &x, &y, &w, &h, path) != 5) return;
    if (strncmp(path, "/run/user/", 10) || strstr(path, "..")) { fprintf(stderr, "glassine-epdd-desktop: impostor: bad path\n"); return; }
    if (w <= 0 || h <= 0 || x < 0 || y < 0 || x + w > pol.W || y + h > pol.H) return;
    if (pen_inking || pol.state == S_PAUSED || pol.state == S_ASLEEP) return;
    int64_t t0 = now_ms(), t_file = 0;
    box b = { x, y, x + w, y + h };
    frame fr = { b, PW - b.y1, b.y1 - b.y0, b.x1 - b.x0, fbuf_get((size_t)w * h), b.y1 - b.y0 };
    const uint8_t *mr;
    int cached = 0;
    if (!fr.g || imp_load(path, w, h, fr.g, &mr, &t_file, &cached) < 0) {
        fbuf_put(fr.g);
        fprintf(stderr, "glassine-epdd-desktop: impostor: cannot read %s\n", path);
        return;
    }
    int64_t t_load = now_ms();
    pthread_mutex_lock(&buf_mu);
    for (int ppx = b.x0; ppx < b.x1; ppx++) {       /* outside the shape: what the panel shows */
        size_t o = (size_t)(ppx - b.x0) * fr.w;
        const uint8_t *sh = shadow + (size_t)ppx * PW + fr.bx0;
        for (int i = 0; i < fr.w; i++) if (mr[o + i]) fr.g[o + i] = sh[i];
    }
    pthread_mutex_unlock(&buf_mu);
    int64_t t_fill = now_ms();
    int64_t t_commit = paint_frame(dpy, root, &fr, 1, out);
    int64_t t_end = now_ms();
    vlog("impostor %dx%d+%d+%d %s: %lld ms (file %lld, load %lld, fill %lld, commit %lld, updates %lld)\n", w, h, x, y,
         cached ? "cached" : "converted", (long long)(t_end - t0), (long long)t_file, (long long)(t_load - t0),
         (long long)(t_fill - t_load), (long long)(t_commit - t_fill), (long long)(t_end - t_commit));
    fbuf_put(fr.g);
}

/* ---- workspace snapshots --------------------------------------------------------
 * xfwm4 unmaps the windows of the workspaces not shown, and X drops their
 * pixels: after a switch every app repaints, one after another, and the panel
 * follows window by window. Leaving a workspace, the panel buffer (what it shows
 * of it) is kept; switching back, it is painted at once (as the burst of a
 * window change: sized, no motion run, the ghost cleanup after) and the apps'
 * repaints then only correct what changed meanwhile. The switch is seen as
 * _NET_CURRENT_DESKTOP changing on the root, from whoever asked for it. A
 * change of the number of workspaces drops the snapshots (glassine-shell
 * renumbers them). */
#define MAXDESK 16
static uint8_t *desk_snap[MAXDESK];
static int cur_desk = -1, n_desk = -1;
/* After a snapshot paint X is not read until the apps are done: a window just
 * mapped shows the wallpaper until its app has drawn, and every such half state
 * differs from the snapshot (a switch back to Firefox painted ~15
 * updates over 2 s, snapshot or not). Reads wait until no damage came for
 * SETTLE_QUIET_MS, at most SETTLE_MAX_MS after the switch; then one read, and
 * only what really differs from the snapshot is painted. */
#define SETTLE_QUIET_MS 200
#define SETTLE_MAX_MS 1500
#define SETTLE_MIN_AREA (64 * 64)       /* smaller damage does not count as a repaint */
/* "rotate" (glassine-rotd, right before sway turns the output): the same hold, longer
 * (an app's resize can come 300+ ms after sway's first turned frame), and its
 * end is ONE full GC16 of the finished screen instead of a read: the panel
 * keeps the old picture until then (user the turn painted sway's
 * unresized frame, then the apps, then the full flash, sometimes before the
 * last app repaint) */
#define ROT_QUIET_MS 400
#define ROT_MAX_MS 2500
static int settle_rot;                  /* this hold is a rotation's */
static int settle_quiet(void) { return settle_rot ? ROT_QUIET_MS : SETTLE_QUIET_MS; }
static int settle_max(void) { return settle_rot ? ROT_MAX_MS : SETTLE_MAX_MS; }

/* 1 while reads still wait; ends the wait when due */
static int settle_hold(int64_t now)
{
    if (!settling) return 0;
    if (now - settle_dmg >= settle_quiet() || now - settle_t0 >= settle_max()) {
        vlog("settled after %lld ms\n", (long long)(now - settle_t0));
        settling = 0;
        pol_settled(&pol, now);
        if (settle_rot) {               /* the turned screen, whole, in one flash */
            settle_rot = 0;
            pol.want_full = 1;
            fprintf(stderr, "glassine-epdd-desktop: rotate: settled after %lld ms\n", (long long)(now - settle_t0));
        }
        return 0;
    }
    return 1;
}

/* ms until the wait may end (-1: none) */
static int settle_timeout(int64_t now)
{
    if (!settling) return -1;
    int64_t a = settle_dmg + settle_quiet(), b = settle_t0 + settle_max(), dt = (a < b ? a : b) - now;
    return dt < 0 ? 0 : (int)dt;
}

/* ---- tile handle drags ------------------------------------------------------
 * dragging a glassine-winfit handle moved its X window on every step; each
 * step became an X read and a GC16 partial (~0.5 s of waveform per step, piling
 * up), and the release (windows resized, the handle placed) a flash, two
 * partials and a full GC16 later. The handle only slides along one axis and
 * looks the same wherever it is, so glassine-epdd-desktop paints it itself:
 * drag start XID X Y W H winfit has drawn the dragged look: those pixels
 * are the bar; reads leave the handle window out
 * from now on (the tiles' gap shows through at its
 * X place) and show the bar where it is dragged
 * drag move X Y the bar's new place: one read of old + new place,
 * dithered DU (A2 on the engine)
 * drag end winfit resizes the windows and puts the handle
 * at its end place in X: like a workspace switch,
 * reads wait until X is quiet, then ONE GC16 partial
 * winfit only does this when the root carries _GLASSINE_EPDD_DRAG (set here while
 * compositing: without it reads cannot leave a window out). A drag nobody ends
 * (winfit died) ends by itself after DRAG_IDLE_MS. */
#define DRAG_IDLE_MS 10000

static void drag_end(void)
{
    if (!dg.on) return;
    dg.on = 0;
    pol_motion(&pol, 0);
    comp_skip(cm, 0);
    fbuf_put(dg.bar.g);
    dg.bar.g = NULL;
    settling = 1;
    settle_t0 = settle_dmg = now_ms();
}

static void drag_move(Display *dpy, Window root, int x, int y)
{
    box nb = { x, y, x + (dg.cur.x1 - dg.cur.x0), y + (dg.cur.y1 - dg.cur.y0) };
    if (nb.x0 == dg.cur.x0 && nb.y0 == dg.cur.y0) return;
    box u = box_clip(box_union(dg.cur, nb), (box){ 0, 0, DisplayWidth(dpy, DefaultScreen(dpy)), DisplayHeight(dpy, DefaultScreen(dpy)) });
    dg.cur = nb;
    int64_t t0 = now_ms();
    frame f = snap(dpy, root, u, src_fb);       /* the gap where it was, the bar where it is */
    if (!f.g) return;
    boxlist ch = {0};
    cm_motion = 1;                              /* (a drag: motion) */
    dz_prep(&f, 1, C_DITHER);
    pthread_mutex_lock(&buf_mu);
    scan(&f, 0, &ch);
    commit(&f, 0, C_DITHER);
    cm_motion = 0;
    pthread_mutex_unlock(&buf_mu);
    dz_prep_drop();
    fbuf_put(f.g);
    list_fold(&ch, 3);
    actions o = {0};
    for (int i = 0; i < ch.n && o.n < MAXACT; i++) o.a[o.n++] = (action){ A_SEND, ch.b[i], WF_DU, M_PART, 0 };
    run_actions(dpy, root, &o);
    vlog("drag %d,%d: %lld ms\n", x, y, (long long)(now_ms() - t0));
}

/* the FIFO's "drag ..." lines (several may come in one read: the last move wins) */
static void drag_cmd(Display *dpy, Window root, char *cmd)
{
    int mv = 0, mx = 0, my = 0;
    for (char *l = strtok(cmd, "\n"); l; l = strtok(NULL, "\n")) {
        unsigned long xid; int x, y, w, h;
        if (sscanf(l, "drag start %lx %d %d %d %d", &xid, &x, &y, &w, &h) == 5 && cm && w > 0 && h > 0) {
            if (dg.on) drag_end();
            mv = 0;
            box b = box_clip((box){ x, y, x + w, y + h }, (box){ 0, 0, DisplayWidth(dpy, DefaultScreen(dpy)), DisplayHeight(dpy, DefaultScreen(dpy)) });
            if (box_empty(b)) continue;
            dg.bar = snap(dpy, root, b, src_fb);        /* before dg.on: X's pixels */
            if (!dg.bar.g) continue;
            dg.win = (Window)xid;
            dg.cur = dg.orig = b;
            dg.on = 1;
            pol_motion(&pol, 1);
            comp_skip(cm, dg.win);
            vlog("drag start %dx%d+%d+%d\n", w, h, x, y);
        } else if (sscanf(l, "drag move %d %d", &x, &y) == 2 && dg.on) {
            mv = 1; mx = x; my = y;
        } else if (!strncmp(l, "drag end", 8)) {
            if (mv) { drag_move(dpy, root, mx, my); mv = 0; }
            drag_end();
            vlog("drag end\n");
        }
        dg.last = now_ms();
    }
    if (mv) drag_move(dpy, root, mx, my);
}

/* the FIFO's window hints, Wayland source only : what top_event
 * learns from X's Map/Unmap/Configure, sent by glassine-swayd (sway IPC window
 * events) and glassine-shell-wl (its popups). Portrait px, one per line:
 * win X Y W H a window appeared / went / moved: its area (pol_window)
 * win new X Y W H a new app window: that, and reads settle as on MapNotify
 * popup X Y W H a popup mapped (pol_popup: a fast first paint)
 * popup gone X Y W H a popup unmapped (pol_popup_gone)
 * shell ID X Y W H a shell surface (bar, popup) mapped or moved: calm there
 * shell gone ID unmapped; shell reset: the shell (re)started, none mapped */
static int is_hint(const char *l)
{
    return !strncmp(l, "win ", 4) || !strncmp(l, "popup ", 6) || !strncmp(l, "shell ", 6) || !strncmp(l, "app", 3) || !strncmp(l, "active", 6) ||
           !strncmp(l, "ink ", 4);
}

/* the hint lines of a FIFO read are handled here; the other lines stay in cmd
 * (in order) for the usual handling: one read may hold several writers' lines */
static ssize_t hint_cmd(char *cmd)
{
    int64_t t = now_ms();
    static char rest[4096];                  /* = the FIFO read buffer */
    char *save = NULL;
    rest[0] = 0;
    size_t rl = 0;
    for (char *l = strtok_r(cmd, "\n", &save); l; l = strtok_r(NULL, "\n", &save)) {
        int x, y, w, h, kind;
        if (!is_hint(l)) {
            if (rl < sizeof rest) rl += snprintf(rest + rl, sizeof rest - rl, "%s\n", l);
            continue;
        }
        int nc = 0;
        if (sscanf(l, "apps %d %d %d %d", &x, &y, &w, &h) == 4) {
            nwlapp = 0;
            wl_area = box_clip((box){ x, y, x + w, y + h }, (box){ 0, 0, PH, PW });
            continue;
        }
        if (sscanf(l, "app %d %d %d %d %n", &x, &y, &w, &h, &nc) == 4 && nc && nwlapp < MAXWL) {
            wlapp[nwlapp].b = box_clip((box){ x, y, x + w, y + h }, (box){ 0, 0, PH, PW });
            snprintf(wlapp[nwlapp].cls, sizeof wlapp[0].cls, "%s", l + nc);
            class_publish(wlapp[nwlapp].cls);
            nwlapp++;
            continue;
        }
        if (!strncmp(l, "ink ", 4)) {
            ink_line_cmd(l);
            continue;
        }
        if (!strncmp(l, "shell ", 6)) {         /* by the shell's surface id: a lost or turned "gone" can't strand a box */
            unsigned id;
            if (!strcmp(l, "shell reset")) nshell = 0;
            else if (sscanf(l, "shell gone %x", &id) == 1 || sscanf(l, "shell %x %d %d %d %d", &id, &x, &y, &w, &h) == 5) {
                for (int i = 0; i < nshell; i++)
                    if (shellid[i] == id) {
                        memmove(&shellr[i], &shellr[i + 1], (nshell - i - 1) * sizeof shellr[0]);
                        memmove(&shellid[i], &shellid[i + 1], (nshell - i - 1) * sizeof shellid[0]);
                        nshell--; break;
                    }
                if (strncmp(l, "shell gone", 10)) {
                    box b = box_clip((box){ x, y, x + w, y + h }, (box){ 0, 0, PH, PW });
                    if (nshell == MAXSHELL) vlog("hint shell %x: %d boxes, dropped\n", id, MAXSHELL);
                    else if (!box_empty(b)) { shellr[nshell] = b; shellid[nshell++] = id; }
                }
            }
            vlog("hint %s (%d shell boxes)\n", l, nshell);
            continue;
        }
        if (!strncmp(l, "active", 6)) {
            snprintf(wl_active, sizeof wl_active, "%s", l[6] == ' ' ? l + 7 : "");
            wl_check_active();
            continue;
        }
        if (sscanf(l, "win new %d %d %d %d", &x, &y, &w, &h) == 4) kind = 1;
        else if (sscanf(l, "win %d %d %d %d", &x, &y, &w, &h) == 4) kind = 0;
        else if (sscanf(l, "popup gone %d %d %d %d", &x, &y, &w, &h) == 4) kind = 3;
        else if (sscanf(l, "popup %d %d %d %d", &x, &y, &w, &h) == 4) kind = 2;
        else continue;
        box b = box_clip((box){ x, y, x + w, y + h }, (box){ 0, 0, PH, PW });    /* portrait */
        if (box_empty(b)) continue;
        if (kind <= 1) pol_window(&pol, b, t);
        else if (kind == 2) pol_popup(&pol, b, t);
        else pol_popup_gone(&pol, b, t);
        if (kind == 1 && !pen_inking) {
            settling = 1; settle_t0 = settle_dmg = t;
            vlog("window mapped (hint): settling\n");
        }
        vlog("hint %s %dx%d+%d+%d\n", kind == 0 ? "win" : kind == 1 ? "win new" : kind == 2 ? "popup" : "popup gone", w, h, x, y);
    }
    if (rl >= sizeof rest) rl = sizeof rest - 1;
    memcpy(cmd, rest, rl + 1);
    return (ssize_t)rl;
}

static long root_card(Display *dpy, Window root, Atom a)
{
    Atom type; int fmt; unsigned long n, after; unsigned char *data = NULL;
    long v = -1;
    if (XGetWindowProperty(dpy, root, a, 0, 1, False, XA_CARDINAL, &type, &fmt, &n, &after, &data) == Success &&
        n == 1 && data)
        v = *(long *)data;
    if (data) XFree(data);
    return v;
}

static void desk_forget(void)
{
    for (int i = 0; i < MAXDESK; i++) { free(desk_snap[i]); desk_snap[i] = NULL; }
}

static void desk_changed(Display *dpy, Window root, Atom cur_a, Atom num_a, actions *out)
{
    long nd = root_card(dpy, root, num_a), d = root_card(dpy, root, cur_a);
    if (nd != n_desk) {
        if (n_desk >= 0) vlog("workspaces: %ld (were %d): snapshots dropped\n", nd, n_desk);
        desk_forget();
        n_desk = (int)nd;
    }
    if (d == cur_desk) return;
    int64_t t0 = now_ms();
    int old = cur_desk;
    cur_desk = (int)d;
    if (old < 0 || old >= MAXDESK || d < 0 || d >= MAXDESK) return;
    if (pol.state == S_PAUSED || pol.state == S_ASLEEP) return;
    /* the one left: as the panel shows it */
    if (!desk_snap[old] && !(desk_snap[old] = malloc(sizeof shadow))) return;
    pthread_mutex_lock(&buf_mu);
    memcpy(desk_snap[old], shadow, sizeof shadow);
    pthread_mutex_unlock(&buf_mu);
    if (!desk_snap[d] || pen_inking) { vlog("workspace %d -> %ld: no snapshot\n", old, d); return; }
    /* the whole screen: the frame layout is the panel buffer's */
    box b = { 0, 0, pol.W, pol.H };
    frame fr = { b, PW - b.y1, b.y1 - b.y0, b.x1 - b.x0, desk_snap[d], b.y1 - b.y0 };
    pol_window(&pol, b, t0);                    /* a window change: no run, ghost cleanup after */
    paint_frame(dpy, root, &fr, 0, out);
    settling = 1;
    settle_t0 = settle_dmg = now_ms();
    vlog("workspace %d -> %ld: snapshot painted, %lld ms\n", old, d, (long long)(now_ms() - t0));
}

/* our own path (glassine-epdd-desktop, or glassine-epdd-desktop-wl under sway), read again from disk:
 * a freshly deployed binary takes over */
static void self_path(char *self, size_t n)
{
    ssize_t sn = readlink("/proc/self/exe", self, n - 1);
    if (sn > 0) {
        self[sn] = 0;
        char *del = strstr(self, " (deleted)");
        if (del && !del[10]) *del = 0;
    } else
        snprintf(self, n, "/usr/sbin/glassine-epdd-desktop");
}

/* a reexec tears the engine, the compositor and the socket down before the
 * exec: one that then fails leaves no engine behind (the
 * GlassineOS deployment: execv EACCES, the session restarted). So a child
 * execs the path first (GLASSINE_EPDD_EXECPROBE: main returns at once): 0 = it
 * runs, else the errno */
static char xself[256];
static int xerr_;
static int exec_probe(const char *self, char **argv)
{
    extern char **environ;
    static char *env[512];              /* (built here: the child of a threaded fork only execs) */
    int n = 0;
    for (char **e = environ; *e && n < 510; e++) env[n++] = *e;
    env[n++] = "GLASSINE_EPDD_EXECPROBE=1";
    env[n] = NULL;
    pid_t p = fork();
    if (p < 0) return errno;
    if (!p) {
        execve(self, argv, env);
        _exit(errno & 0x7f ? errno & 0x7f : 126);
    }
    int st = 0;
    while (waitpid(p, &st, 0) < 0 && errno == EINTR) {}
    return WIFEXITED(st) ? WEXITSTATUS(st) : EIO;
}

/* a failed exec's evidence (self is readlink(/proc/self/exe),
 * not the fixed path; does it differ, e.g. an overlay backing or a detached
 * mount's path?): both paths' access and stat, and the raw link */
static void exec_diag(const char *what, const char *self, int err)
{
    char raw[512];
    ssize_t rn = readlink("/proc/self/exe", raw, sizeof raw - 1);
    raw[rn > 0 ? rn : 0] = 0;
    fprintf(stderr, "glassine-epdd-desktop: %s: errno %d (%s); /proc/self/exe \"%s\" (%zd)\n", what, err, strerror(err), raw, rn);
    const char *ps[2] = { self, "/usr/sbin/glassine-epdd-desktop" };
    for (int i = 0; i < 2; i++) {
        struct stat sb;
        int a = access(ps[i], X_OK), ae = a ? errno : 0, s = stat(ps[i], &sb), se = s ? errno : 0;
        fprintf(stderr, "glassine-epdd-desktop: %s:   \"%s\" (%zu): access X_OK %s, stat %s dev %llx ino %llu mode %o\n", what, ps[i], strlen(ps[i]),
                a ? strerror(ae) : "ok", s ? strerror(se) : "ok", s ? 0ull : (unsigned long long)sb.st_dev,
                s ? 0ull : (unsigned long long)sb.st_ino, s ? 0u : (unsigned)sb.st_mode);
    }
}

int main(int argc, char **argv)
{
    if (getenv("GLASSINE_EPDD_EXECPROBE")) return 0;     /* (exec_probe's child: the exec worked) */
    main_th = pthread_self();
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    if (argc > 1 && !strcmp(argv[1], "-vv")) verbose = 2;
    set_gamma(1.8);
    pol_init(&pol, PH, PW);
    mo_init(&mo, PW);
    pol.log = vlog;
    if (getenv("GLASSINE_EPDD")) apply_cmd(getenv("GLASSINE_EPDD"));
    /* the frame source: X (the default), or sway's glassine-epd backend (see wl) */
    const char *srcv = getenv("GLASSINE_EPDD_SRC");
    int use_wl = srcv && !strcmp(srcv, "wl");
    Display *dpy = NULL;
    int scr = 0;
    Window root = None;
    int W = PH, H = PW;
    int dev = 0, derr, fev, ferr;
    if (use_wl) {
        wlsrc = malloc((size_t)PW * PH);
        if (!wlsrc || !(wl = epd_wl_open(NULL, W, H, wl_rect, NULL))) return 1;
        epd_wl_set_dmabuf(wl, &wl_dma_ops, NULL);
        epd_wl_set_grey(wl, wl_grey);
        epd_wl_set_cursor(wl, ptr_cb, NULL);
        epd_wl_set_luts(wl, lut, lut_lum);
        band_push();
        wl_lazy_set(wl_lazy);
        memset(wlsrc, lut[255], (size_t)PW * PH);  /* until the first frame: white */
        wl_base = wlsrc;
    } else {
    dpy = XOpenDisplay(NULL);
    if (!dpy) { fprintf(stderr, "glassine-epdd-desktop: cannot open display\n"); return 1; }
    XSetErrorHandler(xerr);
    fcntl(ConnectionNumber(dpy), F_SETFD, FD_CLOEXEC);
    scr = DefaultScreen(dpy);
    root = RootWindow(dpy, scr);
    W = DisplayWidth(dpy, scr), H = DisplayHeight(dpy, scr);
    if (W != PH || H != PW) {
        fprintf(stderr, "glassine-epdd-desktop: X screen must be portrait %dx%d (got %dx%d) — set fbdev Option \"Rotate\"\n", PH, PW, W, H);
        return 1;
    }
    if (!XDamageQueryExtension(dpy, &dev, &derr) || !XFixesQueryExtension(dpy, &fev, &ferr) ) {
        fprintf(stderr, "glassine-epdd-desktop: need DAMAGE and XFIXES\n");
        return 1;
    }

    /* no pointer sprite: Xorg draws its software cursor into the framebuffer,
     * which XGetImage hides but /dev/fb0 contains (srccheck: the only 53
     * differing pixels) — it would show on the panel. It never did before (the
     * XGetImage path), and on e-ink a pointer that moves with the pen is just a
     * trail of repaints. Lasts while we are connected. */
    XFixesHideCursor(dpy, root);
    }
    int fb = open("/dev/fb0", O_RDWR | O_CLOEXEC);
    ebc = open("/dev/ebc", O_RDWR | O_CLOEXEC);
    if (ebc >= 0) {
        ebuf = mmap(NULL, (size_t)PW * PH * 4, PROT_READ | PROT_WRITE, MAP_SHARED, ebc, 0);
        if (ebuf == MAP_FAILED) { perror("mmap /dev/ebc"); return 1; }
    } else if (errno == ENOENT) {
        /* mainline: the userspace engine. hv=1: the panel driver raises the
         * MAX17135 rails for each burst of frames (GLASSINE_EPDD_HV=0: dry run) */
        const char *hv = getenv("GLASSINE_EPDD_HV");
        char wbf[256];
        epd_waveform_path(EPD_ADAPTER_CONF, wbf, sizeof wbf);   /* (env, epd.conf, old path) */
        const char *gpu = getenv("GLASSINE_EPDD_GPU");
        epd_stream_gpu = !gpu ? 0 : !strcmp(gpu, "check") ? 2 : !strcmp(gpu, "pad") ? 3 : atoi(gpu);
        const char *gpe = getenv("GLASSINE_EPDD_GPE");
        epd_stream_gpe = !gpe ? 0 : !strcmp(gpe, "check") ? 2 : atoi(gpe);
        const char *gte = getenv("GLASSINE_EPDD_GTARGET");
        gt_mode = !gte ? 0 : !strcmp(gte, "check") ? 2 : atoi(gte);
        {                               /* (the update job's env, before the workers start) */
            static const char *const ek[][2] = { { "GLASSINE_UPDATE_JOB", "update_job" }, { "GLASSINE_UPDATE_CPUS", "update_cpus" },
                                                 { "GLASSINE_UPDATE_RT", "update_rt" }, { "GLASSINE_WORKER_RT", "worker_rt" } };
            for (int i = 0; i < 4; i++) {
                const char *e = getenv(ek[i][0]);
                char c[128];
                if (e && *e) { snprintf(c, sizeof c, "%s=%s", ek[i][1], e); upd_parse(c); epd_knobs_ctl(c, NULL, NULL); }
            }
        }
        claim_state_conf();             /* (eink.conf's @claim_state=: before the engine opens) */
        eng = epd_stream_open(wbf, battery_celsius(), hv ? atoi(hv) : 1);
        if (!eng) { fprintf(stderr, "glassine-epdd-desktop: no /dev/ebc and no engine\n"); return 1; }
        pol.has_engine = 1;             /* motion=adaptive: WF_DU4, A2 cuts */
        /* the engine knobs set before it existed (GLASSINE_EPDD) */
        epd_stream_set_matched(eng, eng_matched);
        epd_stream_set_matched_rule(eng, eng_matched_from, eng_matched_cap);
        epd_stream_set_nonblock(eng, eng_nonblock);
        epd_stream_set_heal(eng, eng_heal, eng_heal_from);
        epd_stream_set_heal_len(eng, eng_heal_len);
        epd_stream_set_model(eng, eng_model, eng_track);
        epd_stream_set_snap(eng, eng_snap);
        eng_temp_ms = now_ms();
        boost_set(0);                   /* a boost left by a crash / reexec */
        boosts = 0;
        if (getenv("GLASSINE_EPDD_REEXEC") && !epd_stream_load_levels(eng, ENG_LEVELS))
            fprintf(stderr, "glassine-epdd-desktop: engine: panel levels restored\n");
        fprintf(stderr, "glassine-epdd-desktop: no /dev/ebc: userspace engine over DRM\n");
        /* quiet_ms saved the stock driver's ~47 ms kernel CPU per update; an
         * engine submit costs microseconds (GLASSINE_EPDD / the FIFO can set it) */
        if (!getenv("GLASSINE_EPDD") || !strstr(getenv("GLASSINE_EPDD"), "quiet_ms=")) apply_cmd("quiet_ms=0");
        /* scrolls: fixed A2/2 with payback (user, the default);
         * eink.conf's "@" lines (glassine-settings) and GLASSINE_EPDD override it */
        if (!getenv("GLASSINE_EPDD") || !strstr(getenv("GLASSINE_EPDD"), "motion=")) apply_cmd("motion=adaptive mmin=a2_2 mmax=a2_2");
        drive_prof_apply();             /* (drive_prof= from GLASSINE_EPDD / eink.conf before the stream was open) */
        upd_apply();                    /* (update_job= from GLASSINE_EPDD / eink.conf / GLASSINE_UPDATE_*) */
        /* SIGTERM/SIGINT: finish the running updates and take the rails
         * down (a killed stream would leave them up until the driver's
         * watchdog); poll returns EINTR, the main loop exits */
        struct sigaction sa = { .sa_handler = on_quit };
        sigaction(SIGTERM, &sa, NULL);
        sigaction(SIGINT, &sa, NULL);
    } else { perror("/dev/ebc"); return 1; }
    if (fb_open(fb) < 0 || wl) src_fb = 0;
    if (dpy) {
    Visual *vis = DefaultVisual(dpy, scr);
    if (vis->red_mask && vis->green_mask && vis->blue_mask) {
        fb_rsh = __builtin_ctzl(vis->red_mask); fb_gsh = __builtin_ctzl(vis->green_mask); fb_bsh = __builtin_ctzl(vis->blue_mask);
    }
    shm_init(dpy, scr, W, H);
    }

    actions out = {0};
    pol_recut(&pol, now_ms(), &out);
    run_actions(dpy, root, &out);
    Damage damage = None;
    XserverRegion parts = None;
    if (dpy) {
    XSelectInput(dpy, root, PropertyChangeMask | SubstructureNotifyMask);   /* before comp_open lists the windows */
    /* the GPU path when X has one; else XDamage on the root. GLASSINE_EPDD_NOCOMP=1: never */
    cm = comp_open(dpy, root, W, H, PW, PH);
    if (cm) comp_set_lut(cm, lut);
    {   /* winfit leaves handle drags to us only while we composite (drag_cmd) */
        Atom a_drag = XInternAtom(dpy, "_GLASSINE_EPDD_DRAG", False);
        long one = 1;
        if (cm) XChangeProperty(dpy, root, a_drag, XA_CARDINAL, 32, PropModeReplace, (unsigned char *)&one, 1);
        else XDeleteProperty(dpy, root, a_drag);
    }
    damage = cm ? None : XDamageCreate(dpy, root, XDamageReportNonEmpty);
    parts = XFixesCreateRegion(dpy, NULL, 0);
    }

    /* root:<panel group> 0660 (video: epd-group.h): the desktop user's "Refresh screen" launcher
     * and sway write it as members of that group (like eink.conf); any other user can't. Made 0600 first so it
     * is never open wider than that; an older image's 0666 FIFO is narrowed too. No such
     * group: root only */
    {
        gid_t vg = epd_group(0);           /* (GLASSINE_EPD_GROUP, default video: epd-group.h) */
        mkfifo(CTL, 0600);
        if (chown(CTL, (uid_t)-1, vg)) perror(CTL);
        chmod(CTL, 0660);
        epd_prof_dir_make(EPD_PROF_DIR, vg);        /* (the CSVs and shots go there) */
    }
    int ctl = open(CTL, O_RDWR | O_NONBLOCK | O_CLOEXEC);

    /* same init sequence as glassine-epd-show */
    if (fb >= 0 && !getenv("GLASSINE_EPDD_REEXEC")) {    /* not on reexec: the unblank also resets the touch controller */
        ioctl(fb, FBIOBLANK, FB_BLANK_POWERDOWN);
        ioctl(fb, FBIOBLANK, FB_BLANK_UNBLANK);
    }

    int xi_op = 0, xi_ev, xi_err, xi_major = 2, xi_minor = 0;
    Atom net_active = None, net_clients = None, net_desk = None, net_ndesk = None;
    if (dpy) {
    {                                   /* the windows already there */
        Window r, par, *kids = NULL;
        unsigned int nk = 0;
        if (XQueryTree(dpy, root, &r, &par, &kids, &nk))
            for (unsigned int i = 0; i < nk; i++) top_get(dpy, kids[i]);
        if (kids) XFree(kids);
    }
    /* any key or button (keyboards, mice, Onboard's XTEST keys) = is
     * active: the quiet limit (epd-policy.c min_gap) must not delay typing.
     * Raw events reach us whoever has the focus; no motion (pen hover would
     * wake us ~200 times a second). Touch and pen come from evdev below. */
    if (XQueryExtension(dpy, "XInputExtension", &xi_op, &xi_ev, &xi_err) &&
        XIQueryVersion(dpy, &xi_major, &xi_minor) == Success) {
        unsigned char mb[XIMaskLen(XI_LASTEVENT)] = {0};
        XISetMask(mb, XI_RawKeyPress);
        XISetMask(mb, XI_RawButtonPress);
        XIEventMask em = { XIAllMasterDevices, sizeof mb, mb };
        XISelectEvents(dpy, root, &em, 1);
    } else xi_op = 0;
    net_active = XInternAtom(dpy, "_NET_ACTIVE_WINDOW", False);
    net_clients = XInternAtom(dpy, "_NET_CLIENT_LIST", False);
    net_desk = XInternAtom(dpy, "_NET_CURRENT_DESKTOP", False);
    net_ndesk = XInternAtom(dpy, "_NET_NUMBER_OF_DESKTOPS", False);
    desk_changed(dpy, root, net_desk, net_ndesk, &out);     /* where we are: no paint */
    }
    unlink(CLASSES);
    load_profiles();
    if (dpy) check_active(dpy, root); else epd_wl_ask_swayd();
    touch_open();
    int64_t pen_input_seen = 0;
    ink_init();
    pthread_t pen_th;
    if (pipe(pen_pipe) || fcntl(pen_pipe[0], F_SETFL, O_NONBLOCK) || fcntl(pen_pipe[1], F_SETFL, O_NONBLOCK) ||
        pthread_create(&pen_th, NULL, pen_thread, NULL)) {
        perror("glassine-epdd-desktop: pen thread");
        return 1;
    }

    boxlist pend = {0};                    /* damage not yet sent */
    boxlist tapd = {0};                    /* a debounce hold's damage near the tap: goes on at once */
    pol.whole = wl != NULL;                /* sway hands over whole frames: no coalesce wait in runs */
    int64_t first_dmg = 0;
    double first_dmg_t = 0;             /* (drive_prof's end-to-end stamp; sway frames only) */
    fprintf(stderr, "glassine-epdd-desktop: running on %dx%d, ctl %s, frames from %s\n", W, H, CTL, wl ? "sway (glassine-epd)" : "X");
    main_tid = syscall(SYS_gettid);     /* (the threads made so far keep the default) */
    main_thread = pthread_self();
    main_sched_apply();

    for (;;) {
        if (wl && epd_wl_dispatch(wl)) {       /* sway's frames: already in wlsrc */
            for (int i = 0; i < wl_dmg.n; i++) {
                if (!pend.n) { first_dmg = now_ms(); first_dmg_t = tsec(); }
                list_add(&pend, wl_dmg.b[i]);
            }
            pol_damage(&pol, now_ms());
            for (int i = 0; i < wl_dmg.n; i++) {
                box c;
                if (pol_debounce_clip(&pol, wl_dmg.b[i], now_ms(), &c)) list_add(&tapd, c);
                /* the shell's own surfaces never wait: they have no transitions */
                if (pol_debounce_left(&pol, now_ms()) > 0)
                    for (int k = 0; k < nshell; k++) {
                        c = box_clip(wl_dmg.b[i], shellr[k]);
                        if (!box_empty(c)) list_add(&tapd, c);
                    }
            }
            if (settling) {
                long a = 0;
                for (int i = 0; i < wl_dmg.n; i++) a += box_area(wl_dmg.b[i]);
                if (a >= SETTLE_MIN_AREA) settle_dmg = now_ms();
            }
        }
        wl_dmg.n = 0;
        while (dpy && XPending(dpy)) {
            XEvent ev;
            XNextEvent(dpy, &ev);
            if (ev.type == GenericEvent && xi_op && ev.xcookie.extension == xi_op) {
                on_input();
                continue;
            }
            if (cm) {                           /* window damage + structure: what to recomposite */
                boxlist d = {0};
                int mine = comp_event(cm, &ev, &d);
                for (int i = 0; i < d.n; i++) {
                    if (!pend.n) first_dmg = now_ms();
                    list_add(&pend, d.b[i]);
                }
                if (d.n) pol_damage(&pol, now_ms());
                if (d.n && settling) {          /* only real repaints delay the settle, not a
                                                 * blinking cursor or scrollbar thumb (those
                                                 * came every 100 ms and held it to SETTLE_MAX_MS) */
                    long a = 0;
                    for (int i = 0; i < d.n; i++) a += box_area(d.b[i]);
                    if (a >= SETTLE_MIN_AREA) settle_dmg = now_ms();
                }
                if (mine) continue;
            }
            if (ev.type == MapNotify || ev.type == UnmapNotify || ev.type == ConfigureNotify ||
                ev.type == DestroyNotify) {
                if (ev.xany.window == root) top_event(dpy, &ev);    /* event window = the parent */
                continue;
            }
            if (ev.type == PropertyNotify) {
                if (ev.xproperty.atom == net_clients) nwcache = 0;   /* ids get reused */
                if (ev.xproperty.atom == net_active) check_active(dpy, root);
                if (ev.xproperty.atom == net_desk || ev.xproperty.atom == net_ndesk)
                    desk_changed(dpy, root, net_desk, net_ndesk, &out);
                continue;
            }
            if (ev.type != dev + XDamageNotify) continue;
            XDamageSubtract(dpy, damage, None, parts);
            int n = 0;
            XRectangle *r = XFixesFetchRegion(dpy, parts, &n);
            for (int i = 0; i < n; i++) {
                box b = { r[i].x, r[i].y, r[i].x + r[i].width, r[i].y + r[i].height };
                b = box_clip(b, (box){ 0, 0, W, H });
                if (box_empty(b)) continue;
                if (!pend.n) first_dmg = now_ms();
                list_add(&pend, b);
                pol_damage(&pol, now_ms());
            }
            if (r) XFree(r);
        }

        /* X: 127-byte reads as always; wl: up to PIPE_BUF, so the window hints
         * (several writers, atomic writes) aren't cut in the middle of a line */
        char cmd[4096];
        ssize_t k = ctl >= 0 ? read(ctl, cmd, dpy ? 127 : sizeof cmd - 1) : -1;
        if (k > 0) cmd[k] = 0;
        /* window hints (Wayland only: X has its own window events), first: they may
         * share a read with other lines */
        if (k > 0 && !dpy) k = hint_cmd(cmd);
        if (k > 0 && dpy && is_hint(cmd)) k = 0;   /* X has its own window events */
        if (k > 0 && !dpy && (!strncmp(cmd, "drag ", 5) || !strncmp(cmd, "impostor ", 9))) k = 0;   /* X only */
        if (k > 0 && !strncmp(cmd, "drag ", 5)) {
            drag_cmd(dpy, root, cmd);           /* alone: several may come at once */
            k = 0;
        }
        if (dg.on && now_ms() - dg.last > DRAG_IDLE_MS) { vlog("drag: no end, ending it\n"); drag_end(); }
        if (k > 0 && !strncmp(cmd, "impostor ", 9)) {
            impostor(dpy, root, cmd, &out);     /* alone: its path must not match other commands */
            k = 0;
        }
        if (k > 0) {
            if (strstr(cmd, "verbose")) verbose = !verbose;
            if (strstr(cmd, "reload")) prof_mt.tv_sec = -2;
            if (strstr(cmd, "park=on")) park_on = 1;
            if (strstr(cmd, "park=off")) park_on = 0;
            const char *gv = strstr(cmd, "gpu=");
            if (gv) setenv("GLASSINE_EPDD_GPU", !strncmp(gv + 4, "on", 2) ? "1" : !strncmp(gv + 4, "check", 5) ? "2" :
                                           !strncmp(gv + 4, "pad", 3) ? "3" : "0", 1);
            const char *ge = strstr(cmd, "gpe=");
            if (ge) setenv("GLASSINE_EPDD_GPE", !strncmp(ge + 4, "on", 2) ? "1" : !strncmp(ge + 4, "check", 5) ? "2" : "0", 1);
            if (ge && !gv) gv = ge;     /* reexecs as gpu= does */
            const char *gt = strstr(cmd, "gtarget=");
            if (gt) setenv("GLASSINE_EPDD_GTARGET", !strncmp(gt + 8, "on", 2) ? "1" : !strncmp(gt + 8, "check", 5) ? "2" : "0", 1);
            if (gt && !gv) gv = gt;
            const char *dv = strstr(cmd, "dry=");
            if (dv) setenv("GLASSINE_EPDD_HV", !strncmp(dv + 4, "on", 2) ? "0" : "1", 1);
            if (dv && eng) epd_stream_set_hv(eng, strncmp(dv + 4, "on", 2) != 0);  /* live, no reexec */
            /* bench=start: glassine-epd-bench takes this process over (an exec as for
             * reexec), drives its test and execs us back (glassine-epd-bench.c) */
            const char *bench = NULL;
            if (strstr(cmd, "bench=start")) bench = eng ? "/usr/sbin/glassine-epd-bench" : NULL;
            if (strstr(cmd, "bench=start") && !bench) fprintf(stderr, "glassine-epdd-desktop: bench: the userspace engine only\n");
            if ((strstr(cmd, "reexec") || gv || bench) && eng && epd_stream_gpu_dead(eng)) {
                fprintf(stderr, "glassine-epdd-desktop: no reexec: the GPU hung and is abandoned (until its frame finishes, else reboot)\n");
                gv = NULL; bench = NULL;
            } else if (bench && access(bench, X_OK)) {
                fprintf(stderr, "glassine-epdd-desktop: bench: %s: %s\n", bench, strerror(errno));
            } else if ((strstr(cmd, "reexec") || gv || bench || (dv && !eng)) &&
                       (self_path(xself, sizeof xself), (xerr_ = exec_probe(xself, argv)) != 0)) {
                fprintf(stderr, "glassine-epdd-desktop: no reexec: %s does not exec (%s): this one keeps running\n", xself, strerror(xerr_));
                exec_diag("reexec probe (a forked child)", xself, xerr_);
            } else if (strstr(cmd, "reexec") || gv || bench || (dv && !eng)) {
                /* replace the binary in place: same PID, so glassine-session (which
                 * ends the desktop when epdd exits) keeps running */
                fprintf(stderr, "glassine-epdd-desktop: reexec \"%s\" (the probe ran it)\n", xself);
                setenv("GLASSINE_EPDD_REEXEC", "1", 1);
                live_export();
                eng_close();
                comp_close(cm);                 /* the new one takes over the windows */
                epd_wl_close(wl);               /* and the socket: sway's backend reconnects */
                wl = NULL;
                char *self = xself;             /* (self_path, probed above) */
                if (bench) {                      /* glassine-epd-bench serve SELF ARGS...: it execs SELF ARGS... back */
                    static char *la[64];
                    int n = 0;
                    la[n++] = "glassine-epd-bench"; la[n++] = "serve"; la[n++] = self;
                    for (int i = 1; argv[i] && n < 63; i++) la[n++] = argv[i];
                    la[n] = NULL;
                    fprintf(stderr, "glassine-epdd-desktop: bench: handing over to %s\n", bench);
                    execv(bench, la);
                    perror("glassine-epdd-desktop: bench");    /* (then back in as a plain reexec) */
                }
                execv(self, argv);
                exec_diag("reexec (the main process, after the probe passed)", self, errno);
                if (cm && (cm = comp_open(dpy, root, W, H, PW, PH))) comp_set_lut(cm, lut);
                if (use_wl && !(wl = epd_wl_open(NULL, W, H, wl_rect, NULL))) return 1;
                epd_wl_set_dmabuf(wl, &wl_dma_ops, NULL);
                epd_wl_set_grey(wl, wl_grey);
                if (wl) epd_wl_set_luts(wl, lut, lut_lum);
                band_push();
                if (wl) wl_lazy_set(wl_lazy);
            }
            const char *tv = strstr(cmd, "temp=");
            if (tv) {
                temp_override = !strncmp(tv + 5, "auto", 4) ? -100 : atoi(tv + 5);
                eng_temp_ms = 0;                /* the next update reads it */
                fprintf(stderr, "glassine-epdd-desktop: waveform temperature %s\n", temp_override > -100 ? tv + 5 : "from the battery");
            }
            if (strstr(cmd, "state")) fprintf(stderr, "glassine-epdd-desktop: state %s\n", state_name(pol.state));
            if (strstr(cmd, "sfnow") && eng) {  /* the model's shortfall right now, nothing reset (poll it during a scroll) */
                double sfm; long sfo;
                epd_stream_shortfall(eng, 3, &sfm, &sfo);
                fprintf(stderr, "%6lld sfnow %s %.2f screens/s mean %.3f frames, %ld px >= 3 short\n", (long long)(now_ms() % 1000000),
                        state_name(pol.state), exp(pol.m_mu), sfm, sfo);
            }
            if (strstr(cmd, "mark")) fprintf(stderr, "%6lld mark\n", (long long)(now_ms() % 1000000));   /* latency tests: a t0 in this log's clock */
            if (strstr(cmd, "penbench")) {
                pthread_t th;
                if (!pthread_create(&th, NULL, pen_bench, NULL)) pthread_detach(th);
            }
            if (strstr(cmd, "stats")) {
                fprintf(stderr, "glassine-epdd-desktop: grabs %lu: read avg %.1f max %.1f ms, scan+commit avg %.1f ms; "
                        "engine hand-offs %lu: avg %.2f ms; input boosts %lu (boost=%d); pace=%s lead %.1f ms\n", tm.n, tm.n ? tm.snap * 1e3 / tm.n : 0, tm.snap_max * 1e3,
                        tm.n ? tm.commit * 1e3 / tm.n : 0, tm.sub, tm.sub ? tm.submit * 1e3 / tm.sub : 0, boosts, boost_ms,
                        pol.pace ? "engine" : "fixed", lead_ms);
                fprintf(stderr, "glassine-epdd-desktop: motion=%s attack=%s mmin=%d mmax=%d steps gc16 %lu du4 %lu a2 %lu a2/8 %lu a2/6 %lu a2/4 %lu a2/2 %lu, estimate %.2f screens/s\n",
                        pol.adaptive ? "adaptive" : "binary", pol.m_attack ? "jump" : "fused", pol.m_min, pol.m_max, pol.lvl_n[LV_GC16], pol.lvl_n[LV_DU4], pol.lvl_n[LV_A2],
                        pol.lvl_n[LV_A2_8], pol.lvl_n[LV_A2_6], pol.lvl_n[LV_A2_4], pol.lvl_n[LV_A2_2], exp(pol.m_mu));
                fprintf(stderr, "glassine-epdd-desktop: motion estimate: %lu bursts, %lu measured (%.2f ms avg), %lu shifted; %lu gestures found by the shift before the finger's slop; %lu cleanups despite small motion (clean_motion=%g); %lu busy-content bursts (busy=%g)\n",
                        mo.bursts, shift_n, shift_n ? shift_ms / shift_n : 0, mo.shifts, mo.early, pol.early_cleans, pol.clean_motion,
                        pol.busy_bursts, pol.busy_min);
                pthread_mutex_lock(&buf_mu);
                fprintf(stderr, "glassine-epdd-desktop: pen brush segments %lu: avg %.1f max %.1f us, halo %s flushes %lu\n", pst.n,
                        pst.n ? pst.us / pst.n : 0, pst.max_us, __atomic_load_n(&ink_halo, __ATOMIC_RELAXED) ? "on" : "off", pst.halo_n);
                memset(&pst, 0, sizeof pst);
                pthread_mutex_unlock(&buf_mu);
                char kl[2048];                  /* the knobs (epd-knobs.h) as they are now */
                epd_knobs_line(epd_ctl_knobs, EPD_CTL_KNOBS_N, kl, sizeof kl);
                fprintf(stderr, "glassine-epdd-desktop: knobs: %s\n", kl);
                pol.early_cleans = 0; pol.busy_bursts = 0;
                mo.bursts = mo.shifts = mo.early = 0; shift_n = 0; shift_ms = 0;
                memset(pol.lvl_n, 0, sizeof pol.lvl_n);
                memset(&tm, 0, sizeof tm);
            }
            if (strstr(cmd, "stats") && eng) {  /* since the last "stats"; then reset */
                if (drive_prof == 1) epd_stream_prof_dump(eng, NULL);     /* (the summary + the frames ring as CSV) */
                if (MP_ON) main_prof_print();
                if (update_job) {               /* totals since start */
                    unsigned long long j, w, c, se;
                    epd_stream_upd_stats(eng, &j, &w, &c, &se);
                    fprintf(stderr, "glassine-epdd-desktop: update_job=pool: workers' jobs %llu (parts: workers %llu, callers %llu; %llu serial), "
                            "own pool's jobs %llu (parts: helpers %llu, callers %llu; %llu serial)\n", j, w, c, se,
                            upool.jobs, upool.hparts, upool.cparts, upool.serial);
                    for (int i = 0; i < UPD_ACCT_N; i++) {  /* (update_acct=1: since the last stats) */
                        struct upd_acct *a = &upool.acct[i];
                        if (!a->jobs && !a->parts) continue;
                        const char *nm = a->fn == upd_scan_part ? "scan" : a->fn == upd_commit_part ? "commit"
                                       : i == UPD_ACCT_N - 1 ? "rest" : "levels";
                        fprintf(stderr, "glassine-epdd-desktop: update_acct %s: %llu jobs, %llu parts, CPU %.1f ms (%.3f a job), caller wall %.1f ms (%.3f a job)\n",
                                nm, a->jobs, a->parts, a->cpu_ms, a->jobs ? a->cpu_ms / a->jobs : 0, a->wall_ms, a->jobs ? a->wall_ms / a->jobs : 0);
                        a->jobs = a->parts = 0; a->cpu_ms = a->wall_ms = 0;
                    }
                }
                /* totals since start (silent at soa) */
                int ck = __atomic_load_n(&epd_claim_kernel, __ATOMIC_RELAXED);
                if (ck > 0)
                    fprintf(stderr, "glassine-epdd-desktop: claim_kernel=%s: %lu claims on the arrays' pass (lazy / npre > 4), %lu differ\n",
                            ck == 2 ? "check" : "packed", epd_claim_kernel_soa, epd_claim_kernel_diffs);
                /* the same, the engine's state (read at engine open) */
                const char *cst = getenv("GLASSINE_CLAIM_STATE");
                if (cst && (!strcmp(cst, "packed") || !strcmp(cst, "check")))
                    fprintf(stderr, "glassine-epdd-desktop: claim_state=%s: %lu claims on the arrays' pass (lazy / npre > 4), %lu checked, %lu differ\n",
                            cst, epd_claim_kernel_soa, epd_claim_state_checked, epd_claim_state_diffs);
                /* totals since start (silent with both switches off) */
                if (__atomic_load_n(&epd_claim_bands, __ATOMIC_RELAXED) > 1 || __atomic_load_n(&epd_stream_drive_overlap, __ATOMIC_RELAXED)) {
                    char s4[256];
                    epd_stream_s4(eng, s4, sizeof s4);
                    fprintf(stderr, "glassine-epdd-desktop: %s\n", s4);
                }
                epd_stats es;
                epd_stream_stats(eng, &es);
                fprintf(stderr, "glassine-epdd-desktop: stats flip lag max %.2f ms, event age avg %.2f ms, ", es.lag_ms_max,
                        es.evt_n ? es.evt_age_ms_sum / es.evt_n : 0);
                fprintf(stderr, "frames %llu late %llu render+copy avg %.2f max %.2f ms "
                        "(copy avg %.2f max %.2f ms, %llu KiB/frame), updates %llu "
                        "merged %llu superseded %llu, waits: order %llu pixel %llu slot %llu, peak run %d pend %d, "
                        "links %llu (warm %llu) dead %llu slow frames %llu (+%llu read late) rail drops %llu, gpu frames %llu checked %llu late %llu resets %llu revives %llu, "
                        "event age max %.1f ms, gpu avg %.2f max %.2f ms, check avg %.2f max %.2f ms, "
                        "gpu state %llu: %.1f Mpx, avg %.2f max %.2f ms\n",
                        (unsigned long long)es.frames, (unsigned long long)es.late,
                        es.frames ? es.render_ms_sum / es.frames : 0, es.render_ms_max,
                        es.frames ? es.copy_ms_sum / es.frames : 0, es.copy_ms_max,
                        (unsigned long long)(es.frames ? es.words_copied * 4 / 1024 / es.frames : 0),
                        (unsigned long long)es.updates,
                        (unsigned long long)es.merged, (unsigned long long)es.superseded,
                        (unsigned long long)es.wait_order, (unsigned long long)es.wait_pixel,
                        (unsigned long long)es.wait_slot, es.nrun_max, es.npend_max,
                        (unsigned long long)es.links, (unsigned long long)es.warm_wakes, (unsigned long long)es.dead_links, (unsigned long long)es.slow_frames, (unsigned long long)es.slow_reads,
                        (unsigned long long)es.rail_drops, (unsigned long long)es.gpu_frames,
                        (unsigned long long)es.gpu_checked, (unsigned long long)es.gpu_late, (unsigned long long)es.gpu_resets, (unsigned long long)es.gpu_revives, es.evt_age_ms_max,
                        es.gpu_frames ? es.gpu_ms_sum / es.gpu_frames : 0, es.gpu_ms_max,
                        es.gpu_checked ? es.check_ms_sum / es.gpu_checked : 0, es.check_ms_max,
                        (unsigned long long)es.state_n, es.state_px / 1e6,
                        es.state_n ? es.state_ms_sum / es.state_n : 0, es.state_ms_max);
#define AVG(f, n) (es.n ? es.f##_ms_sum / es.n : 0), es.f##_ms_max
#define JL(i) (unsigned long long)es.jit_n[i], es.jit_n[i] ? es.jit_lead_ms_sum[i] / es.jit_n[i] : 0
                if (es.jit_n[0] + es.jit_n[1] + es.jit_n[2] + es.jit_n[3])
                    fprintf(stderr, "glassine-epdd-desktop: jit_size=%s: frames (avg lead ms) small %llu (%.1f) mid %llu (%.1f) large %llu (%.1f) stagger %llu (%.1f)\n",
                            epd_stream_jit_size ? "on" : "off", JL(0), JL(1), JL(2), JL(3));
#undef JL
                if (es.gpe_claims || es.gpe_queries || es.gpe_syncs)
                    fprintf(stderr, "glassine-epdd-desktop: gpe: %llu claims (%llu lazy) avg %.2f max %.2f ms, %llu blocked tests, "
                            "%llu state checks avg %.1f max %.1f ms, %llu syncs down avg %.1f max %.1f ms\n",
                            (unsigned long long)es.gpe_claims, (unsigned long long)es.gpe_lazy, AVG(gpe, gpe_claims),
                            (unsigned long long)es.gpe_queries, (unsigned long long)es.gpe_checked, AVG(gpe_chk, gpe_checked),
                            (unsigned long long)es.gpe_syncs, AVG(gpe_sync, gpe_syncs));
                if (es.gpe_deferred)
                    fprintf(stderr, "glassine-epdd-desktop: gpe deferred claims: %llu, %llu collected avg %.3f max %.2f ms, %llu settled inside a pass\n",
                            (unsigned long long)es.gpe_deferred, (unsigned long long)es.gpe_collects, AVG(gpe_col, gpe_collects),
                            (unsigned long long)es.gpe_settle_waits);
                if (dzs.n)
                    fprintf(stderr, "glassine-epdd-desktop: dizzy: %lu commits (%lu serial, %lu on the GPU, %lu of those without an upload) avg %.2f max %.2f ms (under buf_mu avg %.2f), %.1f Mpx/s; reuse %s: %lu of %lu tiles (%lu shifted)\n",
                            dzs.n, dzs.serial, dzs.gpu, dzs.gview, dzs.ms / dzs.n, dzs.max, dzs.lk / dzs.n, dzs.ms > 0 ? dzs.px / dzs.ms / 1e3 : 0,
                            dz_reuse ? "on" : "off", dzr.reused, dzr.tiles, dzr.shifted);
                memset(&dzs, 0, sizeof dzs);
                memset(&dzr, 0, sizeof dzr);
                if (es.stag_entries || es.stag_frames)
                    fprintf(stderr, "glassine-epdd-desktop: stagger: %llu runs (on avg %.1f max %.1f ms [cpu %.1f], off avg %.1f max %.1f ms [read %.1f]), "
                            "%llu cut steps taken (+%llu held, %llu given back), ended %llu quiet %llu other, %llu frames avg %.2f max %.2f ms (submit avg %.2f max %.2f)\n",
                            (unsigned long long)es.stag_entries, AVG(stag_enter, stag_entries),
                            es.stag_entries ? es.stag_enter_cpu_ms_sum / es.stag_entries : 0, AVG(stag_exit, stag_exits),
                            es.stag_exits ? es.stag_exit_read_ms_sum / es.stag_exits : 0,
                            (unsigned long long)es.stag_subs, (unsigned long long)es.stag_held, (unsigned long long)es.stag_flushed,
                            (unsigned long long)es.stag_end_quiet, (unsigned long long)es.stag_end_other, (unsigned long long)es.stag_frames, AVG(stag, stag_frames), AVG(stag_sub, stag_frames));
                if (es.stag_warms || es.stag_first)
                    fprintf(stderr, "glassine-epdd-desktop: stagger warm: %llu warms (avg %.2f max %.2f ms, %llu finished pixels resolved), %llu runs started by their first cut (stagger_entry=%s)\n",
                            (unsigned long long)es.stag_warms, AVG(stag_warm, stag_warms), (unsigned long long)es.stag_warm_px,
                            (unsigned long long)es.stag_first, epd_stream_stag_entry ? "first" : "engine");
                if (es.stag_lane_arms)          /* (stagger_state=lanes) */
                    fprintf(stderr, "glassine-epdd-desktop: stagger lanes: %llu arms (%llu without a D clean, %llu FIN words on armed rows, %llu on the pool), acks %.1f frames after an evict on avg, %llu waited out; syncs %.3f ms a frame (%s)\n",
                            (unsigned long long)es.stag_lane_arms, (unsigned long long)es.stag_lane_dskip, (unsigned long long)es.stag_lane_finarmed, (unsigned long long)es.stag_lane_par, es.stag_exits ? (double)es.stag_lane_ack_frames / es.stag_exits : 0,
                            (unsigned long long)es.stag_lane_ack_late, es.stag_frames ? es.stag_lane_sync_ms_sum / es.stag_frames : 0,
                            epd_dmabuf_partial == 1 ? "range" : "whole buffer");
                if (es.stag_evicts)             /* (the run end's row work, the pool's share) */
                    fprintf(stderr, "glassine-epdd-desktop: stagger run ends: %llu evicts avg %.2f max %.2f ms, takeovers avg %.2f max %.2f ms, %llu of their row passes on the pool\n",
                            (unsigned long long)es.stag_evicts, es.stag_evict_ms_sum / es.stag_evicts, es.stag_evict_ms_max,
                            AVG(stag_exit, stag_exits), (unsigned long long)es.stag_lane_xpar);
                if (es.stag_chk_runs)
                    fprintf(stderr, "glassine-epdd-desktop: stagger check: %llu runs, %llu frames compared, %llu differ (%llu pixels), %llu hand-backs differ; "
                            "self-check %llu frames: %llu differ from a CPU step of the GPU's own state, %llu with the state drifted; "
                            "%llu loads, %llu did not land\n",
                            (unsigned long long)es.stag_chk_runs, (unsigned long long)es.stag_chk_frames, (unsigned long long)es.stag_chk_frames_bad,
                            (unsigned long long)es.stag_chk_px_bad, (unsigned long long)es.stag_chk_state_bad,
                            (unsigned long long)es.stag_chk_self, (unsigned long long)es.stag_chk_self_bad, (unsigned long long)es.stag_chk_drift,
                            (unsigned long long)es.stag_chk_loads, (unsigned long long)es.stag_chk_loads_bad);
                if (es.stag_dc_px)
                    fprintf(stderr, "glassine-epdd-desktop: stagger DC: |record| of the runs' pixels avg %.2f before, %.2f after (%llu px)\n",
                            (double)es.stag_dc_in / es.stag_dc_px, (double)es.stag_dc_out / es.stag_dc_px, (unsigned long long)es.stag_dc_px);
#ifdef EPD_GPU
                if (epd_gpu_stag_ms[2] > 0) {
                    fprintf(stderr, "glassine-epdd-desktop: stagger GPU: %.0f passes, avg %.2f ms (+ template %.2f ms)\n",
                            epd_gpu_stag_ms[2], epd_gpu_stag_ms[0] / epd_gpu_stag_ms[2], epd_gpu_stag_ms[1] / epd_gpu_stag_ms[2]);
                    epd_gpu_stag_ms[0] = epd_gpu_stag_ms[1] = epd_gpu_stag_ms[2] = 0;
                }
                if (epd_gpu_bp4_ms[3] > 0) {
                    fprintf(stderr, "glassine-epdd-desktop: stagger ctl 4: %.0f frames, avg pack %.2f visit %.2f drive %.2f ms\n", epd_gpu_bp4_ms[3],
                            epd_gpu_bp4_ms[0] / epd_gpu_bp4_ms[3], epd_gpu_bp4_ms[1] / epd_gpu_bp4_ms[3], epd_gpu_bp4_ms[2] / epd_gpu_bp4_ms[3]);
                    epd_gpu_bp4_ms[0] = epd_gpu_bp4_ms[1] = epd_gpu_bp4_ms[2] = epd_gpu_bp4_ms[3] = 0;
                }
#endif
                epd_dmabuf_sync_sites(stderr);
                if (gt_live || tmt.subs || tmt.cpu)
                    fprintf(stderr, "glassine-epdd-desktop: GPU targets: %lu hand-offs with one (engine %llu), %lu CPU copies, %lu checked equal, "
                            "%lu differed; mirror: %lu commits on the GPU, %lu by upload, %lu uploads %.0f kpx\n",
                            tmt.subs, (unsigned long long)es.gt_subs, tmt.cpu, tmt.chk, tmt.bad, tmt.gcommits, tmt.ucommits,
                            tmt.ups, tmt.uppx / 1e3);
                memset(&tmt, 0, sizeof tmt);
                fprintf(stderr, "glassine-epdd-desktop: step timeline: %llu hand-offs: copy avg %.2f max %.2f, lock wait avg %.2f max %.2f, "
                        "engine avg %.2f max %.2f ms; %llu frames: schedule avg %.2f max %.2f, advance avg %.2f max %.2f ms (under the lock)\n",
                        (unsigned long long)es.sub_n, AVG(sub_make, sub_n), AVG(sub_lock, sub_n), AVG(sub_tgt, sub_n),
                        (unsigned long long)es.sched_n, AVG(sched, sched_n), AVG(adv, sched_n));
#undef AVG
                {
                    unsigned long th, tmiss;
                    int tk;
                    epd_target_pool_stats(&th, &tmiss, &tk);
                    fprintf(stderr, "glassine-epdd-desktop: target pool (target_pool=%d): %lu reused, %lu fresh blocks >= 128 KB, %d kept\n",
                            epd_target_pool, th, tmiss, tk);
                }
                epd_stream_stats_reset(eng);
                int dcm; long dco;
                epd_stream_dc(eng, DC_WARN, &dcm, &dco);
                fprintf(stderr, "glassine-epdd-desktop: dc: max %d net frames, %ld pixels beyond %d (matched %s from %d cap %d payback %s, %llu px, %llu split starts, %llu absorbed, %llu preempted so far; flashes left %llu px alone)\n",
                        dcm, dco, DC_WARN, eng_matched ? "on" : "off", eng_matched_from, eng_matched_cap,
                        eng_nonblock == 2 ? "preempt" : eng_nonblock ? "nonblock" : "wait", epd_stream_matched_px(eng), epd_stream_split_starts(eng), epd_stream_absorbed(eng), epd_stream_preempted(eng), epd_stream_flash_skipped(eng));
                {
                    double sfm; long sfo;
                    epd_stream_shortfall(eng, 3, &sfm, &sfo);
                    fprintf(stderr, "glassine-epdd-desktop: model: %s, heal %d from %d len %d, snap %s, %llu px healed, %llu levels snapped so far; now mean shortfall %.3f frames, %ld px >= 3 short\n",
                            eng_model == 2 ? "panel" : eng_model ? "optical" : eng_track || eng_heal ? "kept" : "off", eng_heal, eng_heal_from, eng_heal_len,
                            eng_snap ? "on" : "off", epd_stream_healed(eng), epd_stream_snapped(eng), sfm, sfo);
                }
                /* where the CPU goes, since the last "stats" (per update: the
                 * engine's updates; per drive frame: the stream's frames) */
                {
                    static double last[4];
                    double now_c[4], d[4];
                    struct timespec ts;
                    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts); now_c[0] = ts.tv_sec + ts.tv_nsec / 1e9;
                    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts); now_c[1] = ts.tv_sec + ts.tv_nsec / 1e9;
                    epd_stream_cpu(eng, &now_c[2], &now_c[3]);
                    for (int k = 0; k < 4; k++) { d[k] = now_c[k] - last[k]; last[k] = now_c[k]; }
                    double nu = es.updates ? (double)es.updates : 1, nf = es.frames ? (double)es.frames : 1;
                    fprintf(stderr, "glassine-epdd-desktop: cpu: %.2f s (main %.2f, drive %.2f, workers %.2f, other %.2f); "
                            "per update %.1f ms (main %.1f), per drive frame: drive %.2f ms, workers %.2f ms\n",
                            d[0], d[1], d[2], d[3], d[0] - d[1] - d[2] - d[3], d[0] * 1e3 / nu, d[1] * 1e3 / nu,
                            d[2] * 1e3 / nf, d[3] * 1e3 / nf);
                }
            }
            if (strstr(cmd, "dcmap") && eng) {  /* where the DC record piles up: |dc| as a PGM + a summary */
                static int16_t dcm[EPD_W * EPD_H];
                static uint8_t lvm[EPD_W * EPD_H], opm[EPD_W * EPD_H];
                int opr = 0;
                epd_stream_dc_map(eng, dcm, lvm, opm, &opr);
                FILE *o = prof_out("glassine-dc.pgm");
                long n = 0, pos = 0, grey = 0, hist[8] = { 0 };
                int x0 = EPD_W, y0 = EPD_H, x1 = -1, y1 = -1;
                if (o) fprintf(o, "P5 %d %d 255\n", EPD_W, EPD_H);
                for (int i = 0; i < EPD_W * EPD_H; i++) {
                    int a = dcm[i] < 0 ? -dcm[i] : dcm[i];
                    if (o) fputc(a > 255 ? 255 : a, o);
                    hist[a >= 256 ? 7 : a >> 5]++;
                    if (a <= DC_WARN) continue;
                    int x = i % EPD_W, y = i / EPD_W;
                    n++; pos += dcm[i] > 0; grey += lvm[i] != 0 && lvm[i] != 30;
                    if (x < x0) x0 = x;
                    if (x > x1) x1 = x;
                    if (y < y0) y0 = y;
                    if (y > y1) y1 = y;
                }
                if (o) fclose(o);
                /* the engine's idea of each pixel's level (0 black .. 30 white; x8 in
                 * the PGM): against the shot's panel buffer, where the model thinks
                 * a pixel is not at its target yet, or wrongly thinks it is */
                if ((o = prof_out("glassine-lv.pgm"))) {
                    fprintf(o, "P5 %d %d 255\n", EPD_W, EPD_H);
                    for (int i = 0; i < EPD_W * EPD_H; i++) fputc(lvm[i] > 31 ? 255 : lvm[i] * 8, o);
                    fclose(o);
                }
                /* the panel model: where the engine thinks each pixel really is
                 * (0 black .. op_r white, scaled to 0..255; 0 everywhere = no model) */
                long opoff = 0;
                if (opr > 0 && (o = prof_out("glassine-op.pgm"))) {
                    fprintf(o, "P5 %d %d 255\n", EPD_W, EPD_H);
                    for (int i = 0; i < EPD_W * EPD_H; i++) {
                        int v = opm[i] > opr ? opr : opm[i];
                        fputc(v * 255 / opr, o);
                        /* level at a rail, the model on the other half */
                        opoff += (lvm[i] == 30 && 2 * v < opr) || (lvm[i] == 0 && 2 * v > opr);
                    }
                    fclose(o);
                }
                fprintf(stderr, "glassine-epdd-desktop: dcmap: %s/glassine-op.pgm (panel model, op_r %d): %ld pixels at a rail level whose model is on the other half\n",
                        EPD_PROF_DIR, opr, opoff);
                fprintf(stderr, "glassine-epdd-desktop: dcmap: %s/glassine-dc.pgm (|dc|, panel order), glassine-lv.pgm (levels x8); |dc| by 32s: %ld %ld %ld %ld %ld %ld %ld %ld; "
                        "%ld beyond %d (%ld positive, %ld now grey) in x %d..%d y %d..%d\n",
                        EPD_PROF_DIR, hist[0], hist[1], hist[2], hist[3], hist[4], hist[5], hist[6], hist[7], n, DC_WARN, pos, grey, x0, x1, y0, y1);
            }
            if (strstr(cmd, "srccheck") && dpy) src_check(dpy, root, W, H);
            if (strstr(cmd, "shot")) {           /* panel-order PGMs: the panel buffer + a fresh full read */
                FILE *o = prof_out("glassine-shot-panel.pgm");
                if (o) {
                    pthread_mutex_lock(&buf_mu);
                    fprintf(o, "P5 %d %d 255\n", PW, PH);
                    fwrite(shadow, 1, sizeof shadow, o);
                    pthread_mutex_unlock(&buf_mu);
                    fclose(o);
                }
                frame f = snap(dpy, root, (box){ 0, 0, W, H }, src_fb);
                if (f.g && (o = prof_out("glassine-shot-src.pgm"))) {
                    fprintf(o, "P5 %d %d 255\n", f.w, f.h);
                    for (int r = 0; r < f.h; r++) fwrite(f.g + (size_t)r * f.s, 1, (size_t)f.w, o);
                    fclose(o);
                }
                fbuf_put(f.g);
                fprintf(stderr, "glassine-epdd-desktop: shot: %s/glassine-shot-panel.pgm (panel buffer), glassine-shot-src.pgm (%s now)\n",
                        EPD_PROF_DIR, wl ? "sway" : cm ? "composited" : "X root");
            }
            if (strstr(cmd, "stats") && wl) {
                unsigned long nf, npre, nstale;
                unsigned long long npx;
                epd_wl_stats(wl, &nf, &npx);
                nstale = epd_wl_stale(wl, &npre);
                fprintf(stderr, "glassine-epdd-desktop: wl: %lu frames (lazy=%s: %lu answered unread, %lu with an old gamma tag dropped), %llu damaged px; "
                        "gamma on %s, %lu read through it; convert %lu rects %llu px "
                        "%.1f ms total, max %.2f ms, %.2f ns/px\n", nf, wl_lazy ? "on" : "off", epd_wl_skipped(wl), nstale, npx,
                        epd_wl_caps(wl) & GLASSINE_EPD_CAP_GAMMA ? "sway's GPU" : "our CPU", npre,
                        tmw.n, tmw.px, tmw.s * 1e3, tmw.max * 1e3, tmw.px ? tmw.s * 1e9 / tmw.px : 0);
                memset(&tmw, 0, sizeof tmw);
                if (band_t || true_n) fprintf(stderr, "glassine-epdd-desktop: deadband=%d: %lu true frames, %lu with something to clean\n", band_t, true_n, true_changed);
                true_n = true_changed = 0;
                fprintf(stderr, "glassine-epdd-desktop: ingest_view=%s: %lu frames read in place (whole-buffer sync avg %.2f ms), "
                        "%lu copies back (avg %.2f ms)%s\n", ingest_view ? "on" : "off", tmv.n, tmv.n ? tmv.sync * 1e3 / tmv.n : 0,
                        tmv.own, tmv.own ? tmv.own_s * 1e3 / tmv.own : 0, wl_vh >= 0 ? "; reading in place now" : "");
                memset(&tmv, 0, sizeof tmv);
                fprintf(stderr, "glassine-epdd-desktop: pointer=%s: %s, %lu paints\n", ptr_mode ? "on" : "off",
                        !ptr.img ? "no sprite" : ptr_vis ? "shown" : "hidden (pen)", ptr_paints);
                ptr_paints = 0;
                unsigned long dg, dc;
                epd_wl_dmabuf_stats(wl, &dg, &dc);
                if (dg || dc || ing || ing_failed)
                    fprintf(stderr, "glassine-epdd-desktop: ingest=%s on %s: dma-buf frames %lu %s (%.1f ms avg, max %.2f, %.2f ns/px), "
                            "%lu the CPU way; checked %lu, %lu differed (%llu px)\n",
                            ingest_mode == 2 ? "check" : ingest_mode ? "gpu" : "cpu",
                            ing ? (epd_ingest_dead(ing) ? "a dead GPU" : epd_ingest_renderer(ing)) : "no GPU",
                            dg, ingest_mode ? "on the GPU" : "mapped", tmg.n ? tmg.s * 1e3 / tmg.n : 0, tmg.max * 1e3, tmg.px ? tmg.s * 1e9 / tmg.px : 0,
                            dc, tmg.chk, tmg.bad, tmg.badpx);
                memset(&tmg, 0, sizeof tmg);
                if (ing) {                      /* totals since start (silent with all three off) */
                    int ik, ib, is;
                    epd_ingest_cpu_switches(&ik, &ib, &is);
                    if (ik || ib > 1 || is >= 0) {
                        char s3c[400];
                        epd_ingest_cpu_stats(ing, s3c, sizeof s3c);
                        fprintf(stderr, "glassine-epdd-desktop: %s\n", s3c);
                    }
                }
                ingapp_stats();
                unsigned long cn;
                double ca, cx;
                epd_wl_chain(wl, &cn, &ca, &cx);
                fprintf(stderr, "glassine-epdd-desktop: wl: done -> next frame %lu: avg %.1f max %.1f ms (ahead=%s ahead_ms=%d, p75 %.0f ms)\n",
                        cn, ca, cx, wl_ahead == 2 ? "predict" : wl_ahead ? "run" : "off", wl_ahead_ms, epd_wl_chain_pct(wl, 75, 0));
            }
            if (strstr(cmd, "stats") && cm) {
                char cs[512];
                comp_stats(cm, cs, sizeof cs);
                fprintf(stderr, "glassine-epdd-desktop: %s (%s)\n", cs, comp_renderer(cm));
            }
            if (strstr(cmd, "bench=")) {         /* 10 full-screen snaps from one source (CPU: see /proc) */
                int fbsrc = !!strstr(cmd, "bench=fb");
                int64_t t0 = now_ms();
                for (int i = 0; i < 10; i++) { frame f = snap(dpy, root, (box){ 0, 0, W, H }, fbsrc); fbuf_put(f.g); }
                fprintf(stderr, "glassine-epdd-desktop: bench %s: 10 full-screen snaps %lld ms\n", fbsrc ? "fb" : "x", (long long)(now_ms() - t0));
            }
            if (strstr(cmd, "rotate") && pol.state != S_ASLEEP && pol.state != S_PAUSED) {
                settling = 1; settle_rot = 1;
                settle_t0 = settle_dmg = now_ms();
            }
            out.n = 0;
            if (strstr(cmd, "sleep") && pol.state != S_ASLEEP) {
                pill_load(pol.W, pol.H);
                pol_banner(&pol, pill_box);
                fprintf(stderr, "glassine-epdd-desktop: sleep: %s\n", pill_px ? "pill" : "bar");
            }
            if (pol_cmd(&pol, cmd, now_ms(), &out)) {
                /* woke: the banner goes, and whatever changed meanwhile comes in */
                boxlist ch = {0}, ignore = {0};
                box chg = {0};
                grab(dpy, root, pol.banner, 1, &ignore);
                grab(dpy, root, (box){ 0, 0, W, H }, 0, &ch);
                for (int i = 0; i < ch.n; i++) chg = box_union(chg, ch.b[i]);
                pol_wake(&pol, chg, now_ms(), &out);
            }
            run_actions(dpy, root, &out);
            /* asleep / paused: the link fully down once the banner is out, no warm
             * idle (a suspend or another panel owner follows) */
            if (eng) epd_stream_park(eng, pol.state == S_ASLEEP || pol.state == S_PAUSED);
            /* gamma/ink (daemon side); the policy's settings are applied again: harmless */
            apply_cmd(cmd);
            live_note(cmd);
        }

        out.n = 0;
        int prof_new = load_profiles();
        static int live_done;
        if (!live_done) {                       /* (after the first eink.conf: the live settings win) */
            live_done = 1;
            const char *lv = getenv("GLASSINE_EPDD_LIVE");
            if (lv && *lv) {
                fprintf(stderr, "glassine-epdd-desktop: live settings kept across the reexec: %s\n", lv);
                apply_cmd(lv);
                live_note(lv);
            }
        }
        if (prof_new) {                         /* E-ink Center saved */
            if (dpy) check_active(dpy, root); else wl_check_active();
            pol_recut(&pol, now_ms(), &out);
        }
        if (eng && epd_stream_levels_lost(eng)) {   /* the GPU hung with the engine state on it */
            fprintf(stderr, "glassine-epdd-desktop: panel levels lost with the GPU: full flash\n");
            pol.want_full = 1;
        }
        touch_recheck();
        if (touch_read()) on_input();
        if (wl) { act_recheck(); if (act_read()) on_input(); }
        pol_finger(&pol, now_ms(), touch_down, finger_speed());
        if (park_at && !touch_down && now_ms() >= park_at) {
            park_at = 0;
            if (park_on && dpy) park(dpy, root, W, H);
        }

        /* the pen thread's news: what it inked (already on its way to the panel)
         * goes to the policy for the cleanup once the pen is gone */
        {
            char tmp[64];
            while (read(pen_pipe[0], tmp, sizeof tmp) > 0) {}
            pthread_mutex_lock(&buf_mu);
            boxlist nb = ink_new;
            ink_new.n = 0;
            pthread_mutex_unlock(&buf_mu);
            for (int i = 0; i < nb.n; i++) pol_inked(&pol, nb.b[i], now_ms());
            bh_sweep();                         /* brushes the strokes let go of */
            if (pen_input_ms > pen_input_seen) { pen_input_seen = pen_input_ms; pol_input(&pol, pen_input_seen); boost(); }
        }
        if (imp_check.n && now_ms() >= imp_check_at) {     /* impostors X has had time to confirm */
            for (int i = 0; i < imp_check.n; i++) {
                if (!pend.n) first_dmg = now_ms();
                list_add(&pend, imp_check.b[i]);
            }
            pol_damage(&pol, now_ms());
            imp_check.n = 0;
        }
        int inking = pen_inking;
        pen_ok = pol.state != S_PAUSED && pol.state != S_ASLEEP;
        /* the pointer shows in this iteration's reads (ptr_stamp) */
        int64_t pen_ago = now_ms() - pen_input_ms;
        ptr_vis = wl && ptr_mode && ptr.img && !inking && pen_ago >= PTR_PEN_MS;

        int64_t t = now_ms();
        /* after a workspace snapshot: damage collects, no read yet */
        int held = settle_hold(t);
        /* a debounce hold: only the damage near the tap is read now; pend comes back after */
        boxlist db_rest;
        int db_swap = 0;
        if (tapd.n && (held || pol_debounce_left(&pol, t) <= 0)) tapd.n = 0;
        if (tapd.n) { db_rest = pend; pend = tapd; pol.db_tap = 1; db_swap = 1; }
        int ew = eng_wait(&pend);
        static int ew0 = -2;            /* verbose: eng_wait when this damage was first seen */
        if (!pend.n) ew0 = -2;
        else if (ew0 == -2) ew0 = ew;
        pol_engine(&pol, ew);
        if (wl_true.n && !inking && !held) true_frames(dpy, root, &pend, &first_dmg, t);
        double tps = MP_ON ? tsec() : 0;
        int st = pol_step(&pol, t, inking, held ? 0 : pend.n, first_dmg, &out);
        double tpe = MP_ON ? tsec() : 0;
        run_actions(dpy, root, &out);
        out.n = 0;
        if (st == STEP_BURST) {
            burst_in in = { .now = t, .pwin = { 0, 0, W, H }, .pm = PM_AUTO };
            int64_t tg = now_ms();
            double tl = tsec();
            int run_burst = pol.state == S_RUN;
            MADD(MF_POLICY, (tpe - tps) * 1e3);
            last_rd = (box){0};
            for (int i = 0; i < pend.n; i++) last_rd = box_union(last_rd, pend.b[i]);
            frame fr[MAXBOX];
            for (int i = 0; i < pend.n; i++) fr[i] = snap(dpy, root, pend.b[i], src_fb);   /* unlocked: slow */
            double tm1 = tsec();
            MADD(MF_SNAP, (tm1 - tl) * 1e3);
            int sh = 0, measured = 0;
            double sc = 0;
            pthread_mutex_lock(&buf_mu);
            memset(tile_chg, 0, sizeof tile_chg);
            for (int i = 0; i < pend.n; i++) if (fr[i].g) scan_job(&fr[i], 0, &in.ch);   /* (in parts with update_job=pool) */
            double tm2 = MP_ON || drive_prof == 1 ? tsec() : 0;
            if (mo_wants_shift(&mo, t)) measured = measure_shift(fr, pend.n, &in.ch, &sh, &sc);
            pthread_mutex_unlock(&buf_mu);
            double tm3 = MP_ON ? tsec() : 0;
            MADD(MF_SCAN, (tm2 - tm1) * 1e3);
            MADD(MF_SHIFT, (tm3 - tm2) * 1e3);
            if (in.ch.n) mo_burst(&mo, t, measured, sh, sc);
            if (verbose)
                fprintf(stderr, "%6lld grab %d rects -> %d changed, %lld ms (dmg age %lld, first dmg %lld; eng_wait %d at first %d, lead %.0f)\n",
                        (long long)(now_ms() % 1000000), pend.n, in.ch.n, (long long)(now_ms() - tg),
                        (long long)(tg - first_dmg), (long long)(first_dmg % 1000000), ew, ew0, lead_ms);
            /* app profile (E-ink Center): the window under the largest changed
             * rect; after a touch gesture its "touch" mode wins */
            int burst_dither = -1;          /* (the app's dither=) */
            if (in.ch.n) {
                box big = in.ch.b[0];
                for (int i = 1; i < in.ch.n; i++) if (box_area(in.ch.b[i]) > box_area(big)) big = in.ch.b[i];
                const profile *pr = &defprof;
                if (dpy) pr = profile_at(dpy, root, (big.x0 + big.x1) / 2, (big.y0 + big.y1) / 2, &in.pwin);
                else pr = wl_profile_at((big.x0 + big.x1) / 2, (big.y0 + big.y1) / 2, &in.pwin);
                /* the motion estimate (epd-motion): moving = a run; the app's
                 * "touch" mode when the finger is part of it; a finger down
                 * with nothing moving = a press (a tap, or the moment before a
                 * scroll: shown at once, not a run) */
                double mv;
                int ms = mo_state(&mo, t, &mv);
                /* pages=: moving only on a content shift (a swipe that
                 * replaces the page is no motion) */
                if (pr->pages && ms == MO_MOVING && t - mo.shift_t > MO_HOLD_MS) ms = mo.down ? MO_PRESS : MO_STILL;
                int tch = ms == MO_MOVING && mo.by_touch;
                in.pm = tch && pr->touch != PM_AUTO ? pr->touch : pr->mode;
                if (ms == MO_PRESS && pr->touch == PM_FAST && in.pm != PM_QUALITY) in.pm = PM_PRESS;
                in.moving = ms == MO_MOVING;
                in.speed = mv;
                in.busy = pr->busy;
                in.always = pr->always;
                in.pages = pr->pages;
                if (pr == &shellprof) {         /* the bar, a shell popup: never motion */
                    in.calm = 1;
                    in.pm = PM_AUTO;
                    in.moving = 0;
                }
                burst_dither = pr->dither >= 0 ? pr->dither : -1;
                in.dither = pr->dither == DITHER_OFF ? -1 : pr->dither >= 0;   /* (its own pattern: dithered even with dither=off) */
                run_range(pr->lo, pr->hi, pr->curve);
                if (verbose) fprintf(stderr, "profile %s%s%s -> %s, window %d,%d-%d,%d; motion %s %.2f screens/s (shift %d score %.2f%s)\n",
                                     pr->match, tch ? " (touch)" : "", pr->always ? " (always in motion)" : pr->pages ? " (pages)" : "", pm_name(in.pm), in.pwin.x0, in.pwin.y0, in.pwin.x1, in.pwin.y1,
                                     ms == MO_MOVING ? "moving" : ms == MO_PRESS ? "press" : "still", mv, sh, sc, measured ? "" : ", not measured");
            }
            int how = pol_burst(&pol, &in, &out);
            double tm4 = MP_ON ? tsec() : 0;
            MADD(MF_POLICY, (tm4 - tm3) * 1e3);
            /* the pixels enter the panel buffer right before the updates that
             * can take them there */
            long left = 0;
            cm_pat = in.ch.n ? burst_dither : -1;
            cm_motion = pol.state == S_RUN;
            /* a stroke that started during the snapshot: X's (older) pixels must
             * not overwrite its ink — this burst is dropped, its damage retried */
            double tm5 = MP_ON ? tsec() : 0;
            dz_prep(fr, pend.n, how);              /* (dizzy's dots: before the lock) */
            double tm6 = MP_ON ? tsec() : 0;
            MADD(MF_DIZZY, (tm6 - tm5) * 1e3);
            pthread_mutex_lock(&buf_mu);
            int stale_burst = pen_inking;
            for (int i = 0; i < pend.n; i++)
                if (fr[i].g && !stale_burst) left += commit_pool(&fr[i], 0, how);
            cm_pat = -1;
            cm_motion = 0;
            pthread_mutex_unlock(&buf_mu);
            double tm7 = MP_ON || drive_prof == 1 ? tsec() : 0;
            MADD(MF_COMMIT, (tm7 - tm6) * 1e3);
            if (drive_prof == 1) {              /* the sends below carry the end-to-end stamps */
                double org[EPD_ORG_N] = { first_dmg_t, mo_fence_t >= first_dmg_t ? mo_fence_t : 0,
                                          mo_ing_t >= first_dmg_t ? mo_ing_t : 0, tm2, tm7 };
                if (first_dmg_t <= 0) org[1] = org[2] = 0;
                epd_stream_set_origin(org);
            }
            dz_prep_drop();
            for (int i = 0; i < pend.n; i++) fbuf_put(fr[i].g);
            if (verbose && left) fprintf(stderr, "DU: %ld grey pixels deferred to the GC16 cleanup\n", left);
            double tm8 = MP_ON ? tsec() : 0;
            if (stale_burst) {
                if (verbose) fprintf(stderr, "burst dropped: pen started inking\n");
                st = STEP_NONE;             /* keeps pend */
            } else
                run_actions(dpy, root, &out);
            out.n = 0;
            if (drive_prof == 1) epd_stream_set_origin(NULL);
            if (MP_ON) {
                double te = tsec();
                mcur[MF_SEND] += (float)((te - tm8) * 1e3);
                mcur[MF_TOTAL] = (float)((te - tl) * 1e3);
                main_prof_close(run_burst ? MT_RUN : MT_STILL, tl);
            }
            /* how long a run's read takes to reach the engine (pace=engine) */
            if (run_burst && pol.state == S_RUN && !stale_burst) {
                double ms = (tsec() - tl) * 1e3;
                lead_ms = ms > lead_ms ? lead_ms * 0.5 + ms * 0.5 : lead_ms * 0.9 + ms * 0.1;   /* rises fast, decays slowly */
            }
        }
        if (db_swap) {
            pol.db_tap = 0;
            if (st != STEP_NONE) tapd.n = 0; else tapd = pend;
            pend = db_rest;
        } else if (st != STEP_NONE && !held) pend.n = 0;   /* held: the damage waits for the settled read */
        /* the pointer where it is now (after the burst: its reads had the stamp) */
        int ptr_t = wl ? ptr_step(now_ms()) : -1;
        if (wl && ptr_mode && ptr.img && !ptr_vis && !inking && pen_ago < PTR_PEN_MS) ptr_t = (int)(PTR_PEN_MS - pen_ago);
        /* all of sway's damage read (or none came): it may draw the next frame */
        /* the frame is in wlsrc once it arrived. Outside runs (and ahead=off) it is
         * answered once its damage was read: the apps draw at the panel's pace.
         * In a run: ahead=run at once, ahead=predict when the next one will
         * likely land right before the next read — also after a read, so the
         * next frame is as fresh as it can be */
        int in_run = wl && wl_ahead && pol.state == S_RUN;
        if (wl && !pend.n && !in_run) { if (epd_wl_pending(wl)) wl_done_at = now_ms(); epd_wl_done(wl); }
        if (in_run && epd_wl_pending(wl) && now_ms() - wl_done_at >= wl_ahead_ms && ahead_due(&pend) <= 0) {
            wl_done_at = now_ms();
            epd_wl_next(wl);                /* lazy: the frame stays readable (else a DONE) */
        }
        /* quiet pacing holds the read: the app keeps drawing at its own rate
         * (the newest frame stays readable), its damage tells the policy
         * whether it is busy content (FreeTube one frame per 2 s) */
        /* (the debounce too: only more damage can say the frame has not settled) */
        if (wl && pend.n && epd_wl_pending(wl) && ((!in_run && pol_quiet_held(&pol, now_ms())) || pol_debounce_left(&pol, now_ms()) > 0)) {
            wl_done_at = now_ms();
            epd_wl_next(wl);
        }

        struct pollfd pf[6 + ACT_MAX] = { { dpy ? ConnectionNumber(dpy) : -1, POLLIN, 0 }, { ctl, POLLIN, 0 }, { pen_pipe[0], POLLIN, 0 } };
        int npf = 3;
        if (touch_fd >= 0) pf[npf++] = (struct pollfd){ touch_fd, POLLIN, 0 };
        for (int i = 0; i < act_n; i++) pf[npf++] = (struct pollfd){ act_fd[i], POLLIN, 0 };
        if (wl) {
            int wfd[2], nw = epd_wl_fds(wl, wfd);
            for (int i = 0; i < nw; i++) pf[npf++] = (struct pollfd){ wfd[i], POLLIN, 0 };
        }
        if (ctl < 0) pf[1].fd = -1;
        pol_engine(&pol, eng_wait(&pend));
        /* (held damage waits for the settle: settle_timeout wakes it, not a 0 ms coalesce) */
        int timeout = pol_timeout(&pol, now_ms(), inking, settling ? 0 : pend.n);
        if (tapd.n && (timeout < 0 || timeout > COALESCE_MS)) timeout = COALESCE_MS;
        if (timeout < 0 || timeout > 5000) timeout = 5000;
        int bt = boost_tick();
        if (bt >= 0 && bt < timeout) timeout = bt;
        if (imp_check.n) {
            int64_t dt = imp_check_at - now_ms();
            if (dt < timeout) timeout = dt < 0 ? 0 : (int)dt;
        }
        int stt = settle_timeout(now_ms());
        if (stt >= 0 && stt < timeout) timeout = stt;
        if (ptr_t >= 0 && ptr_t < timeout) timeout = ptr_t;
        if (wl && wl_ahead && pol.state == S_RUN && epd_wl_pending(wl)) {
            int64_t dt = wl_done_at + wl_ahead_ms - now_ms(), due = ahead_due(&pend);
            if (due > dt) dt = due;
            if (dt < timeout) timeout = dt < 0 ? 0 : (int)dt;
        }
        if (park_at) {
            int64_t dt = park_at - now_ms();
            if (dt < 0) dt = 0;
            if (dt < timeout) timeout = (int)dt;
        }
        if (dpy) XFlush(dpy);
        /* events may already sit in Xlib's queue (read while waiting for an
         * XGetImage reply) — the socket then stays quiet, so never block on it */
        if (dpy && XEventsQueued(dpy, QueuedAlready) > 0) timeout = 0;
        poll(pf, npf, timeout);
        if (MP_ON && tsec() - mprof->t0 >= prof_every) main_prof_print();
        if (quit_sig) { fprintf(stderr, "glassine-epdd-desktop: signal: exit\n"); boost_set(0); comp_close(cm); epd_wl_close(wl); eng_close(); return 0; }
    }
}
