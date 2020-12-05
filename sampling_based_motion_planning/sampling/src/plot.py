from matplotlib import pyplot as plt
import numpy as np

file = open("../build/halton.txt",'r')
all_lines_halton = file.read()
arr_halton = [line.split(" ") for line in all_lines_halton.split("\n")]
arr_halton = arr_halton[:len(arr_halton)-1]
arr_halton = np.array(arr_halton, dtype = float)
plt.plot(arr_halton, 'ro')
plt.show()

file = open("../build/hammersley.txt",'r')
all_lines_hammersley = file.read()
arr_hammersley = [line.split(" ") for line in all_lines_hammersley.split("\n")]
arr_hammersley = arr_hammersley[:len(arr_hammersley)-1]
arr_hammersley = np.array(arr_hammersley, dtype = float)
plt.plot(arr_hammersley, 'ro')
plt.show()