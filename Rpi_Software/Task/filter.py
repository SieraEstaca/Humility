class Filter():

        def __init__(self, sig1, sig2, dt):
                self.sig1_filt = 0.0
                self.sig2_filt = 0.0
                self.moving_average(sig1, sig2, dt)

        def moving_average(self, sig1, sig2, dt):
                self.sig1_filt = self.sig1_filt + dt*(sig1 - self.sig1_filt)
                self.sig2_filt = self.sig2_filt + dt*(sig2 - self.sig2_filt)
                return self.sig1_filt, self.sig2_filt

