lower_bound = float(input("Enter lower bound: "))
upper_bound = float(input("Upper bound: "))

while True:
    try:
        pos = float(input("Enter pos: "))
    except:
        break

    tuned = (upper_bound-pos) / (upper_bound - lower_bound)
    if tuned > 0.5:
        tuned = 1 - tuned;

    print(tuned)

