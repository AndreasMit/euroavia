import matplotlib.pyplot as plt

t = [0, 15]
p = [1000, 1000]

for i in range(1, 21):
    if i % 2 == 0:
        t.append(t[-1] + 2.5)
        p.append(p[-1])
    else:
        t.append(t[-1] + 0.5)
        p.append(p[-1] + 100)

t.append(t[-1] + 2.5)
p.append(1000)

t.append(t[-1] + 5)
p.append(1000)


# print t comma seperated
print(",".join([str(i) for i in t]))
# print p same way but only 1 decimal
print(",".join([f"{i:.0f}" for i in p]))

# print sizes
print(len(t))
print(len(p))

plt.figure()
plt.plot(t, p)
plt.grid()
plt.show()


