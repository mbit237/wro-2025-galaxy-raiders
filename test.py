import pickle

# arr = []
# for x in range(400):
#     arr.append([1.1, 1000])

# p = pickle.dumps(arr)

# print(len(p))
msg_length = 3142
HEADER = 42
l1 = msg_length >> 8
l2 = msg_length & 0xFF
header = bytes([HEADER, l1, l2])

print(len(header))