import numpy as np
import pyqtgraph as pg

T_F = 0.5
FRAME_RATE = 60
SAMPLE_PERIOD = 0.01
LABEL_FONT_SIZE = 8
TICK_FONT_SIZE = 8

NDATA_TIME = 128  # preferably, make it a power of 2 so fft runs faster


class MyPlotWidget(pg.PlotWidget):
    def __init__(self, main, ylim, xvar='time', yvar='angle'):
        super().__init__(parent=main, background='w')

        if xvar == 'time':
            xlabel = 'Time [s]'
            self.n_data = NDATA_TIME
            self.xdata = np.arange(0, self.n_data * SAMPLE_PERIOD, SAMPLE_PERIOD)
        else:
            xlabel = 'Frequency [Hz]'
            self.xdata = np.fft.fftshift(np.fft.fftfreq(NDATA_TIME, d=SAMPLE_PERIOD))
            self.xdata = self.xdata[self.xdata > 0]
            self.n_data = len(self.xdata)
        self.setLabel('bottom', xlabel, **{'font-size': f'{LABEL_FONT_SIZE}px'})

        if yvar == 'angle':
            ylabel = 'Angle [º]'
        elif yvar == 'fft':
            ylabel = 'Power [normalized]'
        elif yvar == 'rate':
            ylabel = 'Angular speed [º/s]'
        else:  # yvar == 'acc':
            ylabel = 'Acceleration [g]'
        self.setLabel('left', ylabel, **{'font-size': f'{LABEL_FONT_SIZE}px'})

        self.ydata = np.zeros(shape=self.xdata.shape)
        self.setYRange(*ylim, padding=0)
        self.showGrid(x=True, y=True)

        self.line_ref = self.plot(self.xdata, self.ydata, pen=pg.mkPen(color='r', width=2))

    def update_plot(self, ydata):
        if len(ydata) == 0:
            return

        # Drop off the first y element, append a new one.
        if len(ydata) >= self.n_data:
            self.ydata = np.array(ydata[len(ydata)-self.n_data:])
        else:
            self.ydata = np.concatenate((self.ydata[len(ydata):], ydata))

        self.line_ref.setData(self.xdata, self.ydata)
