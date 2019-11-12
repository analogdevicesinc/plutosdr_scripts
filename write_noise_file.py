import numpy as np

len = 2**18
B = 2**15 * 0.8
i = np.random.uniform(-B,B,len)
q = np.random.uniform(-B,B,len)

# Write to file
out = open("data.bin", "w")
for idx in range(len):
    out.write(str(int(i[idx])) +", "+ str(int(q[idx])) + "\n")
out.close()
