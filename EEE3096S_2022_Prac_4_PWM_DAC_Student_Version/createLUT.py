from math import sin, pi
import matplotlib.pyplot as plt

def saw(x):
    '''
    Return the saw of x (measured in radians).
    '''
    while(x < 0):
        x+=2*pi
    while(x > 2*pi):
        x-=2*pi
    if(pi > x):
        return x/pi
    else:
        return (x-pi)/pi-1

def tri(x):
    '''
    Return the triangle of x (measured in radians).
    '''
    while(x < 0):
        x+=2*pi
    while(x > 2*pi):
        x-=2*pi

    if x < pi/2:
        return 2*x/pi
    elif x < 3*pi/2:
        x-=pi/2
        return -2*x/pi + 1
    else:
        x-=pi
        return 2*x/pi-2
    


def create_lut(name, func, ns):
    with open(name, 'w') as f:
        strvals = []
        vals = []
        for i in range(0, ns):
            val = int((func(2*pi*i/ns) + 1) * 511.5)
            strvals.append(str(val))
            vals.append(val)
        f.write((', '.join(strvals)))
        plt.plot(list(range(0, ns)), vals)
        plt.show()
    with open(name.replace('.', '_array.'), 'w') as f:
        for i, val in enumerate(vals):
            if(i > 0):
                f.write(', ')
                if(i%15 == 0):
                    f.write('\n')
            f.write(str(val))
            


NS = 60000

create_lut("sine.txt", sin, NS)
create_lut("saw.txt", saw, NS)
create_lut("tri.txt", tri, NS)

