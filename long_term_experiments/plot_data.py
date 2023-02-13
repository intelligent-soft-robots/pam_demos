from argparse import ArgumentParser

import o80_pam
import matplotlib.pyplot as plt

        
def plot_pressures(pressures, idx):
    plt.plot(pressures, label=str(idx))
    

if __name__ == "__main__":
    # parser = ArgumentParser()
    # parser.add_argument("data", type=str)
    # parser.add_argument("--min-temp", type=float)
    # parser.add_argument("--max-temp", type=float)
    # parser.add_argument("--colorbar", action='store_true')
    # args = parser.parse_args()

    # files = glob.glob(args.data + "obs*")
    # file_name = args.data + "temp_repeat"
    # for dof in range(4):
    #     plot_diff_q(files, dof)
    #     plt.show()

    log_path = "/tmp/long_term_1"
    pressures = []
    for observation in o80_pam.read_file(log_path):
        pressures.append([observation.get_observed_states().get(i).get() for i in range(8)])
    for i in range(8):
        plot_pressures([p[i] for p in pressures], i)
    plt.legend()
    plt.show()


