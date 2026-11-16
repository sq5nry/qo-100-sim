"""
Morse keyer: click-shaped keying envelope generated at a low sample rate.
Loops the text forever with a pause between repeats.
"""
import numpy as np
from gnuradio import gr

MORSE = {
    'A': '.-', 'B': '-...', 'C': '-.-.', 'D': '-..', 'E': '.', 'F': '..-.',
    'G': '--.', 'H': '....', 'I': '..', 'J': '.---', 'K': '-.-', 'L': '.-..',
    'M': '--', 'N': '-.', 'O': '---', 'P': '.--.', 'Q': '--.-', 'R': '.-.',
    'S': '...', 'T': '-', 'U': '..-', 'V': '...-', 'W': '.--', 'X': '-..-',
    'Y': '-.--', 'Z': '--..', '0': '-----', '1': '.----', '2': '..---',
    '3': '...--', '4': '....-', '5': '.....', '6': '-....', '7': '--...',
    '8': '---..', '9': '----.', '/': '-..-.', '?': '..--..', '.': '.-.-.-',
    ',': '--..--', '-': '-....-', '=': '-...-', '+': '.-.-.',
}

EDGE_S = 0.005      # keying rise/fall time, s


class blk(gr.sync_block):
    """Morse keying envelope (float, 0..1) at a LOW sample rate (env_rate).
    Interpolate, shift and scale it with native blocks downstream."""

    def __init__(self, text='VVV DE N0CALL', wpm=12.0, env_rate=12500.0, pause_s=3.0):
        gr.sync_block.__init__(self, name='Morse keyer', in_sig=None,
                               out_sig=[np.float32])
        units = []
        for word in str(text).upper().split():
            for ch in word:
                code = MORSE.get(ch)
                if code is None:
                    continue
                for sym in code:
                    units += [1] * (1 if sym == '.' else 3) + [0]
                units += [0, 0]          # 3 units between characters
            units += [0, 0, 0, 0]        # 7 units between words
        if not units:
            units = [0]
        dot = 1.2 / max(float(wpm), 1.0)
        per_unit = max(int(round(dot * env_rate)), 1)
        env = np.repeat(np.array(units, dtype=np.float64), per_unit)
        env = np.concatenate([env, np.zeros(int(pause_s * env_rate) + 1)])
        k = np.hanning(int(EDGE_S * env_rate) + 1)
        self.table = np.convolve(env, k / k.sum(), mode='same').astype(np.float32)
        self.pos = 0

    def work(self, input_items, output_items):
        out = output_items[0]
        n = len(out)
        size = len(self.table)
        idx = (self.pos + np.arange(n)) % size
        out[:] = self.table[idx]
        self.pos = (self.pos + n) % size
        return n
