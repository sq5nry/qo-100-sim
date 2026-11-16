#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: 2.4 GHz -> 10 GHz linear transponder (single B200, 3rd harmonic TX)
# Description: 500 kHz linear transponder with two Morse beacons and a local USB voice channel
# GNU Radio version: 3.10.12.0

from PyQt5 import Qt
from gnuradio import qtgui
from PyQt5 import QtCore
from gnuradio import analog
from gnuradio import blocks
import math
from gnuradio import filter
from gnuradio.filter import firdes
from gnuradio import gr
from gnuradio.fft import window
import sys
import signal
from PyQt5 import Qt
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
from gnuradio.filter import pfb
import qo100_sim_transponder_b200_beacon_hi_key as beacon_hi_key  # embedded python block
import qo100_sim_transponder_b200_beacon_lo_key as beacon_lo_key  # embedded python block
import sip
import threading


def snipfcn_leila_ctl(self):
    # LEILA-style policing of the transponder path:
    #  - finds received signals whose power in a 2.7 kHz slot exceeds leila_thresh_dbfs
    #  - marker: puts a two-tone siren on top of up to 3 offenders
    #  - limiter: attenuates only the offending slots down to threshold + leila_limit_db
    # Detection is done before the limiter (feed-forward), a few times per second.
    import numpy as _np
    from PyQt5 import Qt as _QQt
    _le_mark_box = _QQt.QCheckBox("LEILA marker")
    _le_lim_box = _QQt.QCheckBox("LEILA limiter")
    _le_mark_box.setChecked(True)
    _le_lim_box.setChecked(True)
    _le_lbl = _QQt.QLabel("LEILA: starting")
    _le_row = _QQt.QHBoxLayout()
    for _w in (_le_mark_box, _le_lim_box, _le_lbl):
        _le_row.addWidget(_w)
    _le_row.addStretch(1)
    self.top_layout.addLayout(_le_row)

    _LE_TAPS = 1024       # limiter filter length (fixed delay of 512 samples)
    _LE_SLOT = 9          # bins summed for one signal slot (about 2.7 kHz)
    _LE_HOLD = 8          # frames of peak hold (about 0.4 s)
    _LE_GUARD = 16        # bins blanked around a found signal (about +-5 kHz)
    _LE_RELEASE = 10 ** (1.0 / 20)   # limiter recovery per tick: 1 dB
    _le = {"hist": [], "gain": None, "last": None, "tick": 0, "ident": True}
    _le_marks = (self.leila_mark0, self.leila_mark1, self.leila_mark2)
    _le_win_taps = _np.hamming(_LE_TAPS)

    def _le_identity():
        if not _le["ident"]:
            taps = [0j] * _LE_TAPS
            taps[_LE_TAPS // 2] = 1 + 0j
            self.leila_filter.set_taps(taps)
            _le["ident"] = True
            _le["last"] = None
        _le["gain"] = None

    def _le_tick():
        try:
            x = _np.asarray(self.leila_probe.level(), dtype=_np.complex64)
            n = len(x)
            fs = float(self.samp_rate)
            thr_db = float(self.leila_thresh_dbfs)
            lim_db = thr_db + float(self.leila_limit_db)
            mark_amp = 10 ** (float(self.leila_marker_dbfs) / 20.0)
        except Exception:
            return
        if n < 64 or n % 2:
            return
        _le["tick"] += 1
        w = _np.hanning(n)
        p = _np.abs(_np.fft.fftshift(_np.fft.fft(x * w))) ** 2 / (n * _np.sum(w * w))
        slot = _np.convolve(p, _np.ones(_LE_SLOT), "same")
        hist = _le["hist"]
        hist.append(slot)
        del hist[:-_LE_HOLD]
        held = _np.max(hist, axis=0)

        thr = 10 ** (thr_db / 10.0)
        work = held.copy()
        found = []
        for _ in range(len(_le_marks)):
            k = int(_np.argmax(work))
            if work[k] <= thr:
                break
            found.append(((k - n / 2.0) * fs / n, 10 * _np.log10(work[k] / thr)))
            work[max(0, k - _LE_GUARD):k + _LE_GUARD + 1] = 0.0

        siren = 300.0 if (_le["tick"] // 4) % 2 else -300.0
        for i, src in enumerate(_le_marks):
            if _le_mark_box.isChecked() and i < len(found):
                src.set_frequency(found[i][0] + siren)
                src.set_amplitude(mark_amp)
            else:
                src.set_amplitude(0.0)

        reduction = 0.0
        if _le_lim_box.isChecked():
            lim = 10 ** (lim_db / 10.0)
            target = _np.where(held > lim, _np.sqrt(lim / _np.maximum(held, 1e-20)), 1.0)
            target = _np.minimum.reduce([_np.roll(target, s) for s in range(-4, 5)])
            prev = _le["gain"]
            gain = target if prev is None or len(prev) != n else _np.minimum(target, _np.minimum(prev * _LE_RELEASE, 1.0))
            _le["gain"] = gain
            reduction = -20 * _np.log10(max(float(gain.min()), 1e-6))
            if reduction < 0.05:
                _le_identity()
            else:
                g = gain.reshape(_LE_TAPS, n // _LE_TAPS).min(axis=1)
                last = _le["last"]
                if last is None or _np.max(_np.abs(g - last)) > 0.02:
                    h = _np.fft.fftshift(_np.fft.ifft(_np.fft.ifftshift(g))) * _le_win_taps
                    self.leila_filter.set_taps([complex(v) for v in h])
                    _le["last"] = g
                    _le["ident"] = False
        else:
            _le_identity()

        if found:
            f0, over = max(found, key=lambda t: t[1])
            txt = "LEILA: %d signal(s) over limit, worst +%.1f dB at %+.1f kHz (%.4f MHz)" % (
                len(found), over, f0 / 1e3, (float(self.tx_center) + f0) / 1e6)
            if reduction > 0.05:
                txt += ", limiting %.1f dB" % reduction
            if not (_le_mark_box.isChecked() or _le_lim_box.isChecked()):
                txt += " (monitoring only)"
            _le_lbl.setText(txt)
            _le_lbl.setStyleSheet("color: black; background: orange; font-weight: bold; padding: 2px")
        else:
            _le_lbl.setText("LEILA: no signal over %.1f dBFS" % thr_db)
            _le_lbl.setStyleSheet("color: green; padding: 2px")

    self._leila_timer = _QQt.QTimer()
    self._leila_timer.timeout.connect(_le_tick)
    self._leila_timer.start(50)

def snipfcn_level_ctl(self):
    # Drive-level indication: RX in-band peak and TX (DAC) drive peak/RMS in dBFS,
    # with a 3 s peak hold, colour warning and an overdrive event counter.
    import math as _math, time as _time
    from PyQt5 import Qt as _LQt
    _lv_rx = _LQt.QLabel("RX peak: ?")
    _lv_tx = _LQt.QLabel("TX drive: ?")
    _lv_cnt = _LQt.QLabel("Overdrive events: 0")
    _lv_rst = _LQt.QPushButton("Reset")
    _lv_row = _LQt.QHBoxLayout()
    for _w in (_lv_rx, _lv_tx, _lv_cnt, _lv_rst):
        _lv_row.addWidget(_w)
    _lv_row.addStretch(1)
    self.top_layout.addLayout(_lv_row)
    _lv = {"rx": [], "tx": [], "events": 0, "tx_red": False, "rx_red": False}
    _LV_HOLD_S = 3.0

    def _lv_db(p):
        return 10.0 * _math.log10(p) if p > 1e-12 else -120.0

    def _lv_style(lbl, text, level, warn, clip):
        if level >= clip:
            lbl.setStyleSheet("color: white; background: red; font-weight: bold; padding: 2px")
            text += "  OVERDRIVE"
        elif level >= warn:
            lbl.setStyleSheet("color: black; background: orange; font-weight: bold; padding: 2px")
            text += "  high"
        else:
            lbl.setStyleSheet("color: green; padding: 2px")
        lbl.setText(text)

    def _lv_held(key, value):
        now = _time.time()
        hist = _lv[key]
        hist.append((now, value))
        while hist and now - hist[0][0] > _LV_HOLD_S:
            hist.pop(0)
        return max(v for _, v in hist)

    def _lv_poll():
        try:
            rx = _lv_db(self.rx_peak_probe.level())
            tx = _lv_db(self.tx_peak_probe.level())
            tx_rms = _lv_db(self.tx_rms_probe.level())
        except Exception:
            return
        rx_h = _lv_held("rx", rx)
        tx_h = _lv_held("tx", tx)
        _lv_style(_lv_rx, "RX peak: %6.1f dBFS (3 s max %6.1f)" % (rx, rx_h),
                  rx_h, self.rx_warn_dbfs, self.rx_clip_dbfs)
        _lv_style(_lv_tx, "TX drive peak: %6.1f dBFS, RMS %6.1f (3 s max %6.1f)" % (tx, tx_rms, tx_h),
                  tx_h, self.tx_warn_dbfs, self.tx_clip_dbfs)
        for key, level, clip, name in (("tx_red", tx, self.tx_clip_dbfs, "TX drive"),
                                       ("rx_red", rx, self.rx_clip_dbfs, "RX input")):
            red = level >= clip
            if red and not _lv[key]:
                _lv["events"] += 1
                print("[level] %s overdrive: %.1f dBFS peak" % (name, level))
            _lv[key] = red
        _lv_cnt.setText("Overdrive events: %d" % _lv["events"])

    def _lv_reset():
        _lv["events"] = 0
        _lv["rx"].clear()
        _lv["tx"].clear()

    _lv_rst.clicked.connect(_lv_reset)
    self._lv_timer = _LQt.QTimer()
    self._lv_timer.timeout.connect(_lv_poll)
    self._lv_timer.start(100)

def snipfcn_ref_ctl(self):
    # External 10 MHz reference (GPSDO on the B200 REF input):
    # checkbox to switch internal/external at run time + lock status.
    from PyQt5 import Qt as _Qt
    _ref_box = _Qt.QCheckBox("External 10 MHz ref (GPSDO)")
    _ref_lbl = _Qt.QLabel("Ref: ?")
    _row = _Qt.QHBoxLayout()
    _row.addWidget(_ref_box)
    _row.addWidget(_ref_lbl)
    _row.addStretch(1)
    self.top_layout.addLayout(_row)

    def _ref_set(on):
        src = "external" if on else "internal"
        try:
            self.usrp_tx.set_clock_source(src, 0)
            print("[ref] clock source ->", src)
        except Exception as e:
            print("[ref] set_clock_source failed:", e)
        _ref_poll()

    def _ref_poll():
        try:
            src = self.usrp_tx.get_clock_source(0)
        except Exception:
            src = "?"
        if src != "external":
            _ref_lbl.setText("Ref: internal TCXO")
            _ref_lbl.setStyleSheet("color: gray")
            return
        try:
            locked = self.usrp_tx.get_mboard_sensor("ref_locked", 0).to_bool()
        except Exception:
            locked = None
        if locked:
            _ref_lbl.setText("Ref: external 10 MHz - LOCKED")
            _ref_lbl.setStyleSheet("color: green; font-weight: bold")
        else:
            _ref_lbl.setText("Ref: external 10 MHz - NOT LOCKED")
            _ref_lbl.setStyleSheet("color: red; font-weight: bold")

    try:
        _ref_box.setChecked(self.usrp_tx.get_clock_source(0) == "external")
    except Exception:
        pass
    _ref_box.toggled.connect(_ref_set)
    self._ref_timer = _Qt.QTimer()
    self._ref_timer.timeout.connect(_ref_poll)
    self._ref_timer.start(1000)
    _ref_poll()


def snippets_main_after_init(tb):
    snipfcn_leila_ctl(tb)
    snipfcn_level_ctl(tb)
    snipfcn_ref_ctl(tb)

class qo100_sim_transponder_b200(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "2.4 GHz -> 10 GHz linear transponder (single B200, 3rd harmonic TX)", catch_exceptions=True)
        Qt.QWidget.__init__(self)
        self.setWindowTitle("2.4 GHz -> 10 GHz linear transponder (single B200, 3rd harmonic TX)")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except BaseException as exc:
            print(f"Qt GUI: Could not set Icon: {str(exc)}", file=sys.stderr)
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("gnuradio/flowgraphs", "qo100_sim_transponder_b200")

        try:
            geometry = self.settings.value("geometry")
            if geometry:
                self.restoreGeometry(geometry)
        except BaseException as exc:
            print(f"Qt GUI: Could not restore geometry: {str(exc)}", file=sys.stderr)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.tx_shift = tx_shift = 300e3
        self.tx_harmonic = tx_harmonic = 3
        self.tx_center = tx_center = 10489.75e6
        self.samp_rate = samp_rate = 625e3
        self.beacon_level_db = beacon_level_db = -43
        self.xpdr_gain_db = xpdr_gain_db = 5
        self.voice_slack = voice_slack = 300e-6
        self.voice_on = voice_on = False
        self.voice_offset = voice_offset = 0
        self.voice_level_db = voice_level_db = -20
        self.usrp_args = usrp_args = "type=b200"
        self.tx_warn_dbfs = tx_warn_dbfs = -12
        self.tx_target = tx_target = (tx_center + (tx_harmonic + 1) * tx_shift) / tx_harmonic
        self.tx_prefill_ms = tx_prefill_ms = 100
        self.tx_gain = tx_gain = 89.75
        self.tx_clip_dbfs = tx_clip_dbfs = -2
        self.rx_warn_dbfs = rx_warn_dbfs = -12
        self.rx_lo_offset = rx_lo_offset = 0.75e6
        self.rx_gain = rx_gain = 40
        self.rx_clip_dbfs = rx_clip_dbfs = -3
        self.rx_center = rx_center = 2400.25e6
        self.passband = passband = 500e3
        self.lvl_n2 = lvl_n2 = 100
        self.lvl_n1 = lvl_n1 = int(samp_rate/1000)
        self.leila_thresh_dbfs = leila_thresh_dbfs = beacon_level_db + 3
        self.leila_marker_dbfs = leila_marker_dbfs = beacon_level_db
        self.leila_limit_db = leila_limit_db = 6
        self.leila_fft = leila_fft = 2048
        self.env_interp = env_interp = 50
        self.beacon_wpm = beacon_wpm = 12
        self.beacon_offset = beacon_offset = 245e3
        self.beacon_lo_text = beacon_lo_text = "VVV DE N0CALL LOWER BCN"
        self.beacon_hi_text = beacon_hi_text = "VVV DE N0CALL UPPER BCN"
        self.audio_rate = audio_rate = 48000

        ##################################################
        # Blocks
        ##################################################

        _voice_on_check_box = Qt.QCheckBox("Voice TX on")
        self._voice_on_choices = {True: True, False: False}
        self._voice_on_choices_inv = dict((v,k) for k,v in self._voice_on_choices.items())
        self._voice_on_callback = lambda i: Qt.QMetaObject.invokeMethod(_voice_on_check_box, "setChecked", Qt.Q_ARG("bool", self._voice_on_choices_inv[i]))
        self._voice_on_callback(self.voice_on)
        _voice_on_check_box.stateChanged.connect(lambda i: self.set_voice_on(self._voice_on_choices[bool(i)]))
        self.top_layout.addWidget(_voice_on_check_box)
        self._tx_gain_range = qtgui.Range(0, 89.75, 0.25, 89.75, 200)
        self._tx_gain_win = qtgui.RangeWidget(self._tx_gain_range, self.set_tx_gain, "B200 TX gain (dB)", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._tx_gain_win)
        self._rx_gain_range = qtgui.Range(0, 76, 1, 40, 200)
        self._rx_gain_win = qtgui.RangeWidget(self._rx_gain_range, self.set_rx_gain, "B200 RX gain (dB)", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._rx_gain_win)
        self._beacon_level_db_range = qtgui.Range(-60, -6, 1, -43, 200)
        self._beacon_level_db_win = qtgui.RangeWidget(self._beacon_level_db_range, self.set_beacon_level_db, "Beacon level (dBFS)", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._beacon_level_db_win)
        self._xpdr_gain_db_range = qtgui.Range(-20, 80, 1, 5, 200)
        self._xpdr_gain_db_win = qtgui.RangeWidget(self._xpdr_gain_db_range, self.set_xpdr_gain_db, "Transponder digital gain (dB)", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._xpdr_gain_db_win)
        self.xpdr_gain = blocks.multiply_const_cc(10**(xpdr_gain_db/20.0))
        self.waterfall = qtgui.waterfall_sink_c(
            2048, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            tx_center, #fc
            samp_rate, #bw
            "", #name
            1, #number of inputs
            None # parent
        )
        self.waterfall.set_update_time(0.10)
        self.waterfall.enable_grid(False)
        self.waterfall.enable_axis_labels(True)



        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        colors = [0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.waterfall.set_line_label(i, "Data {0}".format(i))
            else:
                self.waterfall.set_line_label(i, labels[i])
            self.waterfall.set_color_map(i, colors[i])
            self.waterfall.set_line_alpha(i, alphas[i])

        self.waterfall.set_intensity_range(-120, -20)

        self._waterfall_win = sip.wrapinstance(self.waterfall.qwidget(), Qt.QWidget)

        self.top_layout.addWidget(self._waterfall_win)
        self.voice_wav = blocks.wavfile_source('C:/Users/pablos/Downloads/beacon_voice_msg.wav', True)
        self.voice_shift = blocks.rotator_cc(2.0*math.pi*voice_offset/samp_rate)
        self.voice_resamp = pfb.arb_resampler_ccf(
            (samp_rate/audio_rate*(1+voice_slack)),
            taps=firdes.low_pass(64, 64*audio_rate, 4e3, 30e3, window.WIN_BLACKMAN_HARRIS),
            flt_size=64,
            atten=80)
        self.voice_resamp.declare_sample_delay(0)
        self._voice_level_db_range = qtgui.Range(-60, -6, 1, -20, 200)
        self._voice_level_db_win = qtgui.RangeWidget(self._voice_level_db_range, self.set_voice_level_db, "Voice level (dBFS peak)", "counter_slider", float, QtCore.Qt.Horizontal)
        self.top_layout.addWidget(self._voice_level_db_win)
        self.voice_level = blocks.multiply_const_cc((10**(voice_level_db/20.0)) if voice_on else 0)
        self.usrp_tx = uhd.usrp_sink(
            ",".join((usrp_args, '')),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
            "",
        )
        self.usrp_tx.set_clock_source('external', 0)
        self.usrp_tx.set_samp_rate(samp_rate)
        self.usrp_tx.set_time_unknown_pps(uhd.time_spec(0))

        self.usrp_tx.set_center_freq(uhd.tune_request(tx_target, -tx_shift), 0)
        self.usrp_tx.set_antenna("TX/RX", 0)
        self.usrp_tx.set_bandwidth(2.0e6, 0)
        self.usrp_tx.set_gain(tx_gain, 0)
        self.usrp_rx = uhd.usrp_source(
            ",".join((usrp_args, '')),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.usrp_rx.set_clock_source('external', 0)
        self.usrp_rx.set_samp_rate(samp_rate)
        self.usrp_rx.set_time_unknown_pps(uhd.time_spec(0))

        self.usrp_rx.set_center_freq(uhd.tune_request(rx_center, rx_lo_offset), 0)
        self.usrp_rx.set_antenna("RX2", 0)
        self.usrp_rx.set_bandwidth(2.5e6, 0)
        self.usrp_rx.set_gain(rx_gain, 0)
        self.undo_inversion = blocks.conjugate_cc()
        self.tx_win2 = blocks.stream_to_vector(gr.sizeof_float*1, lvl_n2)
        self.tx_win1 = blocks.stream_to_vector(gr.sizeof_float*1, lvl_n1)
        self.tx_rms_probe = blocks.probe_signal_f()
        self.tx_pwr = blocks.complex_to_mag_squared(1)
        self.tx_prefill = blocks.delay(gr.sizeof_gr_complex*1, (int(samp_rate*tx_prefill_ms/1000.0)))
        self.tx_peak_probe = blocks.probe_signal_f()
        self.tx_peak = blocks.max_ff(lvl_n2, 1)
        self.tx_max1 = blocks.max_ff(lvl_n1, 1)
        self.tx_avg = blocks.moving_average_ff((lvl_n1*lvl_n2), (1.0/(lvl_n1*lvl_n2)), 4000, 1)
        self.ssb_filter = filter.fir_filter_fcc(
            1,
            firdes.complex_band_pass(
                2,
                audio_rate,
                300,
                2700,
                200,
                window.WIN_HAMMING,
                6.76))
        self.spectrum = qtgui.freq_sink_c(
            2048, #size
            window.WIN_BLACKMAN_hARRIS, #wintype
            tx_center, #fc
            samp_rate, #bw
            "Transponder output (as heard at 10 GHz)", #name
            1,
            None # parent
        )
        self.spectrum.set_update_time(0.10)
        self.spectrum.set_y_axis((-120), 0)
        self.spectrum.set_y_label('Relative Gain', 'dB')
        self.spectrum.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.spectrum.enable_autoscale(False)
        self.spectrum.enable_grid(True)
        self.spectrum.set_fft_average(0.2)
        self.spectrum.enable_axis_labels(True)
        self.spectrum.enable_control_panel(False)
        self.spectrum.set_fft_window_normalized(False)



        labels = ['', '', '', '', '',
            '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
            1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
            "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
            1.0, 1.0, 1.0, 1.0, 1.0]

        for i in range(1):
            if len(labels[i]) == 0:
                self.spectrum.set_line_label(i, "Data {0}".format(i))
            else:
                self.spectrum.set_line_label(i, labels[i])
            self.spectrum.set_line_width(i, widths[i])
            self.spectrum.set_line_color(i, colors[i])
            self.spectrum.set_line_alpha(i, alphas[i])

        self._spectrum_win = sip.wrapinstance(self.spectrum.qwidget(), Qt.QWidget)
        self.top_layout.addWidget(self._spectrum_win)
        self.rx_win2 = blocks.stream_to_vector(gr.sizeof_float*1, lvl_n2)
        self.rx_win1 = blocks.stream_to_vector(gr.sizeof_float*1, lvl_n1)
        self.rx_pwr = blocks.complex_to_mag_squared(1)
        self.rx_peak_probe = blocks.probe_signal_f()
        self.rx_peak = blocks.max_ff(lvl_n2, 1)
        self.rx_max1 = blocks.max_ff(lvl_n1, 1)
        self.leila_vec = blocks.stream_to_vector(gr.sizeof_gr_complex*1, leila_fft)
        self.leila_probe = blocks.probe_signal_vc(leila_fft)
        self.leila_mark_sum = blocks.add_vcc(1)
        self.leila_mark2 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, 0, 0, 0, 0)
        self.leila_mark1 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, 0, 0, 0, 0)
        self.leila_mark0 = analog.sig_source_c(samp_rate, analog.GR_COS_WAVE, 0, 0, 0, 0)
        self.leila_filter = filter.fft_filter_ccc(1, [0]*512 + [1] + [0]*511, 1)
        self.leila_filter.declare_sample_delay(0)
        self.combine = blocks.add_vcc(1)
        self.chan_filter = filter.fir_filter_ccf(
            1,
            firdes.low_pass(
                1,
                samp_rate,
                (passband/2),
                30e3,
                window.WIN_BLACKMAN,
                6.76))
        self.beacon_lo_shift = blocks.rotator_cc(2.0*math.pi*(-beacon_offset)/samp_rate)
        self.beacon_lo_level = blocks.multiply_const_cc(10**(beacon_level_db/20.0))
        self.beacon_lo_key = beacon_lo_key.blk(text=beacon_lo_text, wpm=beacon_wpm, env_rate=samp_rate/env_interp, pause_s=3.0)
        self.beacon_lo_interp = filter.interp_fir_filter_fff(env_interp, firdes.low_pass(env_interp, samp_rate, 2e3, 4e3))
        self.beacon_lo_interp.declare_sample_delay(0)
        self.beacon_lo_f2c = blocks.float_to_complex(1)
        self.beacon_hi_shift = blocks.rotator_cc(2.0*math.pi*(+beacon_offset)/samp_rate)
        self.beacon_hi_level = blocks.multiply_const_cc(10**(beacon_level_db/20.0))
        self.beacon_hi_key = beacon_hi_key.blk(text=beacon_hi_text, wpm=beacon_wpm, env_rate=samp_rate/env_interp, pause_s=3.0)
        self.beacon_hi_interp = filter.interp_fir_filter_fff(env_interp, firdes.low_pass(env_interp, samp_rate, 2e3, 4e3))
        self.beacon_hi_interp.declare_sample_delay(0)
        self.beacon_hi_f2c = blocks.float_to_complex(1)


        ##################################################
        # Connections
        ##################################################
        self.connect((self.beacon_hi_f2c, 0), (self.beacon_hi_shift, 0))
        self.connect((self.beacon_hi_interp, 0), (self.beacon_hi_f2c, 0))
        self.connect((self.beacon_hi_key, 0), (self.beacon_hi_interp, 0))
        self.connect((self.beacon_hi_level, 0), (self.combine, 2))
        self.connect((self.beacon_hi_shift, 0), (self.beacon_hi_level, 0))
        self.connect((self.beacon_lo_f2c, 0), (self.beacon_lo_shift, 0))
        self.connect((self.beacon_lo_interp, 0), (self.beacon_lo_f2c, 0))
        self.connect((self.beacon_lo_key, 0), (self.beacon_lo_interp, 0))
        self.connect((self.beacon_lo_level, 0), (self.combine, 1))
        self.connect((self.beacon_lo_shift, 0), (self.beacon_lo_level, 0))
        self.connect((self.chan_filter, 0), (self.xpdr_gain, 0))
        self.connect((self.combine, 0), (self.spectrum, 0))
        self.connect((self.combine, 0), (self.tx_pwr, 0))
        self.connect((self.combine, 0), (self.undo_inversion, 0))
        self.connect((self.combine, 0), (self.waterfall, 0))
        self.connect((self.leila_filter, 0), (self.combine, 0))
        self.connect((self.leila_mark0, 0), (self.leila_mark_sum, 0))
        self.connect((self.leila_mark1, 0), (self.leila_mark_sum, 1))
        self.connect((self.leila_mark2, 0), (self.leila_mark_sum, 2))
        self.connect((self.leila_mark_sum, 0), (self.combine, 4))
        self.connect((self.leila_vec, 0), (self.leila_probe, 0))
        self.connect((self.rx_max1, 0), (self.rx_win2, 0))
        self.connect((self.rx_peak, 0), (self.rx_peak_probe, 0))
        self.connect((self.rx_pwr, 0), (self.rx_win1, 0))
        self.connect((self.rx_win1, 0), (self.rx_max1, 0))
        self.connect((self.rx_win2, 0), (self.rx_peak, 0))
        self.connect((self.ssb_filter, 0), (self.voice_resamp, 0))
        self.connect((self.tx_avg, 0), (self.tx_rms_probe, 0))
        self.connect((self.tx_max1, 0), (self.tx_win2, 0))
        self.connect((self.tx_peak, 0), (self.tx_peak_probe, 0))
        self.connect((self.tx_prefill, 0), (self.usrp_tx, 0))
        self.connect((self.tx_pwr, 0), (self.tx_avg, 0))
        self.connect((self.tx_pwr, 0), (self.tx_win1, 0))
        self.connect((self.tx_win1, 0), (self.tx_max1, 0))
        self.connect((self.tx_win2, 0), (self.tx_peak, 0))
        self.connect((self.undo_inversion, 0), (self.tx_prefill, 0))
        self.connect((self.usrp_rx, 0), (self.chan_filter, 0))
        self.connect((self.usrp_rx, 0), (self.rx_pwr, 0))
        self.connect((self.voice_level, 0), (self.combine, 3))
        self.connect((self.voice_resamp, 0), (self.voice_shift, 0))
        self.connect((self.voice_shift, 0), (self.voice_level, 0))
        self.connect((self.voice_wav, 0), (self.ssb_filter, 0))
        self.connect((self.xpdr_gain, 0), (self.leila_filter, 0))
        self.connect((self.xpdr_gain, 0), (self.leila_vec, 0))


    def closeEvent(self, event):
        self.settings = Qt.QSettings("gnuradio/flowgraphs", "qo100_sim_transponder_b200")
        self.settings.setValue("geometry", self.saveGeometry())
        self.stop()
        self.wait()

        event.accept()

    def get_tx_shift(self):
        return self.tx_shift

    def set_tx_shift(self, tx_shift):
        self.tx_shift = tx_shift
        self.set_tx_target((self.tx_center + (self.tx_harmonic + 1) * self.tx_shift) / self.tx_harmonic)
        self.usrp_tx.set_center_freq(uhd.tune_request(self.tx_target, -self.tx_shift), 0)

    def get_tx_harmonic(self):
        return self.tx_harmonic

    def set_tx_harmonic(self, tx_harmonic):
        self.tx_harmonic = tx_harmonic
        self.set_tx_target((self.tx_center + (self.tx_harmonic + 1) * self.tx_shift) / self.tx_harmonic)

    def get_tx_center(self):
        return self.tx_center

    def set_tx_center(self, tx_center):
        self.tx_center = tx_center
        self.set_tx_target((self.tx_center + (self.tx_harmonic + 1) * self.tx_shift) / self.tx_harmonic)
        self.spectrum.set_frequency_range(self.tx_center, self.samp_rate)
        self.waterfall.set_frequency_range(self.tx_center, self.samp_rate)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_lvl_n1(int(self.samp_rate/1000))
        self.beacon_hi_interp.set_taps(firdes.low_pass(self.env_interp, self.samp_rate, 2e3, 4e3))
        self.beacon_hi_shift.set_phase_inc(2.0*math.pi*(+self.beacon_offset)/self.samp_rate)
        self.beacon_lo_interp.set_taps(firdes.low_pass(self.env_interp, self.samp_rate, 2e3, 4e3))
        self.beacon_lo_shift.set_phase_inc(2.0*math.pi*(-self.beacon_offset)/self.samp_rate)
        self.chan_filter.set_taps(firdes.low_pass(1, self.samp_rate, (self.passband/2), 30e3, window.WIN_BLACKMAN, 6.76))
        self.leila_mark0.set_sampling_freq(self.samp_rate)
        self.leila_mark1.set_sampling_freq(self.samp_rate)
        self.leila_mark2.set_sampling_freq(self.samp_rate)
        self.spectrum.set_frequency_range(self.tx_center, self.samp_rate)
        self.tx_prefill.set_dly(int((int(self.samp_rate*self.tx_prefill_ms/1000.0))))
        self.usrp_rx.set_samp_rate(self.samp_rate)
        self.usrp_tx.set_samp_rate(self.samp_rate)
        self.voice_resamp.set_rate((self.samp_rate/self.audio_rate*(1+self.voice_slack)))
        self.voice_shift.set_phase_inc(2.0*math.pi*self.voice_offset/self.samp_rate)
        self.waterfall.set_frequency_range(self.tx_center, self.samp_rate)

    def get_beacon_level_db(self):
        return self.beacon_level_db

    def set_beacon_level_db(self, beacon_level_db):
        self.beacon_level_db = beacon_level_db
        self.set_leila_marker_dbfs(self.beacon_level_db)
        self.set_leila_thresh_dbfs(self.beacon_level_db + 3)
        self.beacon_hi_level.set_k(10**(self.beacon_level_db/20.0))
        self.beacon_lo_level.set_k(10**(self.beacon_level_db/20.0))

    def get_xpdr_gain_db(self):
        return self.xpdr_gain_db

    def set_xpdr_gain_db(self, xpdr_gain_db):
        self.xpdr_gain_db = xpdr_gain_db
        self.xpdr_gain.set_k(10**(self.xpdr_gain_db/20.0))

    def get_voice_slack(self):
        return self.voice_slack

    def set_voice_slack(self, voice_slack):
        self.voice_slack = voice_slack
        self.voice_resamp.set_rate((self.samp_rate/self.audio_rate*(1+self.voice_slack)))

    def get_voice_on(self):
        return self.voice_on

    def set_voice_on(self, voice_on):
        self.voice_on = voice_on
        self._voice_on_callback(self.voice_on)
        self.voice_level.set_k((10**(self.voice_level_db/20.0)) if self.voice_on else 0)

    def get_voice_offset(self):
        return self.voice_offset

    def set_voice_offset(self, voice_offset):
        self.voice_offset = voice_offset
        self.voice_shift.set_phase_inc(2.0*math.pi*self.voice_offset/self.samp_rate)

    def get_voice_level_db(self):
        return self.voice_level_db

    def set_voice_level_db(self, voice_level_db):
        self.voice_level_db = voice_level_db
        self.voice_level.set_k((10**(self.voice_level_db/20.0)) if self.voice_on else 0)

    def get_usrp_args(self):
        return self.usrp_args

    def set_usrp_args(self, usrp_args):
        self.usrp_args = usrp_args

    def get_tx_warn_dbfs(self):
        return self.tx_warn_dbfs

    def set_tx_warn_dbfs(self, tx_warn_dbfs):
        self.tx_warn_dbfs = tx_warn_dbfs

    def get_tx_target(self):
        return self.tx_target

    def set_tx_target(self, tx_target):
        self.tx_target = tx_target
        self.usrp_tx.set_center_freq(uhd.tune_request(self.tx_target, -self.tx_shift), 0)

    def get_tx_prefill_ms(self):
        return self.tx_prefill_ms

    def set_tx_prefill_ms(self, tx_prefill_ms):
        self.tx_prefill_ms = tx_prefill_ms
        self.tx_prefill.set_dly(int((int(self.samp_rate*self.tx_prefill_ms/1000.0))))

    def get_tx_gain(self):
        return self.tx_gain

    def set_tx_gain(self, tx_gain):
        self.tx_gain = tx_gain
        self.usrp_tx.set_gain(self.tx_gain, 0)

    def get_tx_clip_dbfs(self):
        return self.tx_clip_dbfs

    def set_tx_clip_dbfs(self, tx_clip_dbfs):
        self.tx_clip_dbfs = tx_clip_dbfs

    def get_rx_warn_dbfs(self):
        return self.rx_warn_dbfs

    def set_rx_warn_dbfs(self, rx_warn_dbfs):
        self.rx_warn_dbfs = rx_warn_dbfs

    def get_rx_lo_offset(self):
        return self.rx_lo_offset

    def set_rx_lo_offset(self, rx_lo_offset):
        self.rx_lo_offset = rx_lo_offset
        self.usrp_rx.set_center_freq(uhd.tune_request(self.rx_center, self.rx_lo_offset), 0)

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.usrp_rx.set_gain(self.rx_gain, 0)

    def get_rx_clip_dbfs(self):
        return self.rx_clip_dbfs

    def set_rx_clip_dbfs(self, rx_clip_dbfs):
        self.rx_clip_dbfs = rx_clip_dbfs

    def get_rx_center(self):
        return self.rx_center

    def set_rx_center(self, rx_center):
        self.rx_center = rx_center
        self.usrp_rx.set_center_freq(uhd.tune_request(self.rx_center, self.rx_lo_offset), 0)

    def get_passband(self):
        return self.passband

    def set_passband(self, passband):
        self.passband = passband
        self.chan_filter.set_taps(firdes.low_pass(1, self.samp_rate, (self.passband/2), 30e3, window.WIN_BLACKMAN, 6.76))

    def get_lvl_n2(self):
        return self.lvl_n2

    def set_lvl_n2(self, lvl_n2):
        self.lvl_n2 = lvl_n2
        self.tx_avg.set_length_and_scale((self.lvl_n1*self.lvl_n2), (1.0/(self.lvl_n1*self.lvl_n2)))

    def get_lvl_n1(self):
        return self.lvl_n1

    def set_lvl_n1(self, lvl_n1):
        self.lvl_n1 = lvl_n1
        self.tx_avg.set_length_and_scale((self.lvl_n1*self.lvl_n2), (1.0/(self.lvl_n1*self.lvl_n2)))

    def get_leila_thresh_dbfs(self):
        return self.leila_thresh_dbfs

    def set_leila_thresh_dbfs(self, leila_thresh_dbfs):
        self.leila_thresh_dbfs = leila_thresh_dbfs

    def get_leila_marker_dbfs(self):
        return self.leila_marker_dbfs

    def set_leila_marker_dbfs(self, leila_marker_dbfs):
        self.leila_marker_dbfs = leila_marker_dbfs

    def get_leila_limit_db(self):
        return self.leila_limit_db

    def set_leila_limit_db(self, leila_limit_db):
        self.leila_limit_db = leila_limit_db

    def get_leila_fft(self):
        return self.leila_fft

    def set_leila_fft(self, leila_fft):
        self.leila_fft = leila_fft

    def get_env_interp(self):
        return self.env_interp

    def set_env_interp(self, env_interp):
        self.env_interp = env_interp
        self.beacon_hi_interp.set_taps(firdes.low_pass(self.env_interp, self.samp_rate, 2e3, 4e3))
        self.beacon_lo_interp.set_taps(firdes.low_pass(self.env_interp, self.samp_rate, 2e3, 4e3))

    def get_beacon_wpm(self):
        return self.beacon_wpm

    def set_beacon_wpm(self, beacon_wpm):
        self.beacon_wpm = beacon_wpm

    def get_beacon_offset(self):
        return self.beacon_offset

    def set_beacon_offset(self, beacon_offset):
        self.beacon_offset = beacon_offset
        self.beacon_hi_shift.set_phase_inc(2.0*math.pi*(+self.beacon_offset)/self.samp_rate)
        self.beacon_lo_shift.set_phase_inc(2.0*math.pi*(-self.beacon_offset)/self.samp_rate)

    def get_beacon_lo_text(self):
        return self.beacon_lo_text

    def set_beacon_lo_text(self, beacon_lo_text):
        self.beacon_lo_text = beacon_lo_text

    def get_beacon_hi_text(self):
        return self.beacon_hi_text

    def set_beacon_hi_text(self, beacon_hi_text):
        self.beacon_hi_text = beacon_hi_text

    def get_audio_rate(self):
        return self.audio_rate

    def set_audio_rate(self, audio_rate):
        self.audio_rate = audio_rate
        self.ssb_filter.set_taps(firdes.complex_band_pass(2, self.audio_rate, 300, 2700, 200, window.WIN_HAMMING, 6.76))
        self.voice_resamp.set_taps(firdes.low_pass(64, 64*self.audio_rate, 4e3, 30e3, window.WIN_BLACKMAN_HARRIS))
        self.voice_resamp.set_rate((self.samp_rate/self.audio_rate*(1+self.voice_slack)))




def main(top_block_cls=qo100_sim_transponder_b200, options=None):

    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    snippets_main_after_init(tb)
    tb.start()
    tb.flowgraph_started.set()

    tb.show()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        Qt.QApplication.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    timer = Qt.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)

    qapp.exec_()

if __name__ == '__main__':
    main()
