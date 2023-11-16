from collections import deque
import numpy as np

matrix1 = np.array([[1, 2],
                    [3, 4],
                    [5, 6]])

matrix2 = np.array([[5, 6],
                     [3,4],
                     [1,3]])

matrix2_tranpose = np.transpose(matrix2) 
# print(np.matmul(matrix1,matrix2))


# print(matrix2[0][1])
# print(matrix2_tranpose[0][1])

# huy = np.matrix('0.250063290666895	-0.00499975001251790;-0.00231023244887541	0.999950002502598;0.240118104244286	-99.9950002503959;-10.8354240588002	2757.07143215445')

# print(huy)



Wc = np.array([[0], 
                [0], 
                [2]])


Ac = np.array([ [0, 1, 0],
                [0, 0, 1],
                [0, 0, 0]])

# print(np.shape(Ac))
# print(np.identity(np.shape(Ac)[0]))

# Ad = np.eye(len(Ac)) + 0.2 * Ac

# print(Ad)

# Cc = np.array([[1, 0, 0], 
#                 [0, 1, 0]])

# # print(Cc*3)
# print(Cc[1][2])

y = np.array([[0],
                [0]])

# print(y[3])
Qz = np.array([[0.499975001249938,	0],
                    [0.0149932530361350,	0],
                    [-0.5,	33.3333333333388],
                    [2.37988143430733,	-158.730158730185]])
w = np.array([[14],
            [-5],
            [7],
            [10]])
Z = np.matmul(Qz,y) + w
print(Z)

# E = np.concatenate((np.identity(np.shape(Ac)[0]), Wc), axis = 1)
# print(E)

# print(np.zeros([3,1]))


_velocity_buffer = deque(maxlen=3)

for i in range(0,10):
    _velocity_buffer.append(i)
    if i>1:
        print(f"0  : {_velocity_buffer[0]}")
        print(f"1  : {_velocity_buffer[1]}")
        print(f"2  : {_velocity_buffer[2]}")
        print("+++++")
        print(f"-1  : {_velocity_buffer[-1]}")
        print(f"-2  : {_velocity_buffer[-2]}")
        print("-------")

length = len(_velocity_buffer)
print("Length of deque:", length)

# _velocity_buffer = [3]
# _velocity_buffer.append(1)
# _velocity_buffer.append(1)
# _velocity_buffer.append(1)
# _velocity_buffer.append(1)
# print(_velocity_buffer)

# # from collections import deque
# my_deque = deque(maxlen=3)
# # my_deque = deque([1, 2, 3, 4, 5])

# for i in range(1,10):
#     my_deque.append(i)
#     element = my_deque[-1]

# # my_deque.appendleft(0)

# right_element = my_deque.pop()
# left_element = my_deque.popleft()

# element = my_deque[-1]

# length = len(my_deque)

# for item in my_deque:
#     print(item)

# print("Right Element:", right_element)
# print("Left Element:", left_element)
# print("Element at index -1:", element)
# print("Length of deque:", length)

Qz = np.matrix('0.499775101204458	1.22124532708767e-15;\
                    0.0149932530361350	-8.14903700074865e-14;\
                    -0.499775101204540	33.3333333333388;\
                    2.37988143430733	-158.730158730185')
print(Qz)

def huy(nb):
    global Qz
    b = np.matrix('0.0	1.2;\
                    0.1	-1.1;\
                    -0.1	1.1;\
                    2.1	-1.1')
    Qz = b*nb
    print(Qz)

huy(2)
huy(3)
print(Qz)


psi = np.array([[0, 
                    0, 
                    0]])

print(psi[0][0])
print(psi[0][1])