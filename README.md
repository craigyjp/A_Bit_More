## Based on the the original Crumar Bit series synthesizers I have created an RP2040 based DCO synthesizer.

This is an 8 voice bi-timbral polyphonic dual DCO based synthesizer that uses a very similar technology to the original Crumar Bit series synths. An RP2040 is used to create each clock output that if ded to the 4520 counters and resistor ladders to create the sawtooth waveforms.
The RP2040 also generates a sub oscillator, two PWM waves which can be modulated to create PWM. The triangle output is derived from the second sawtooth. I have also added portamento, octave switching, oscillator sync etc.

Filters are based on the Matrix 12 pole switching filters with 16 filter types available, dual LFO's are vailable for modulation and PWM, plus a white/pink noise source.
