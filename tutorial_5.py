import time
import o80
import o80_pam


fm = o80_pam.FileManager()
path = fm.next()
print("\ndata will be saved in {}".format(path))

print("\nsaving data for 5 seconds ...")
with o80_pam.Logger("o80_pam_robot",path):
    time_start = time.time()
    while time.time()-time_start < 5:
        time.sleep(0.01)
print("... done")

print("\nreading the file.")
observations = list(o80_pam.read_file(path))
print("{} observations read\n".format(len(observations)))

