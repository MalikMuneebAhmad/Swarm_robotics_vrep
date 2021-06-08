import matplotlib.pyplot as plt


class Inflammation():

    def __init__(self, init_num_bac):
        self.init_num_bac = init_num_bac  # Initial number of bacteria
        self.pres_bac_inflam = int()
        self.total_inflam = [0]

    def perc_inflam(self, pres_num_bac):
        self.pres_bac_inflam = pres_num_bac / self.init_num_bac
        print('present bacteria inflammation', self.pres_bac_inflam)
        self.total_inflam.append(self.pres_bac_inflam)
        change_inflam = self.total_inflam[-1] - self.total_inflam[-2]
        return self.pres_bac_inflam, change_inflam

    def plot_inflam(self):
        plt.plot(range(len(self.total_inflam)), self.total_inflam)
        plt.show()