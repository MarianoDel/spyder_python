from control import matlab

import matplotlib.pyplot as plt



def stepResponse(Ts,*args,**kwargs):

    num = Poly(Ts.as_numer_denom()[0],s).all_coeffs()

    den = Poly(Ts.as_numer_denom()[1],s).all_coeffs()

    tf = matlab.tf(map(float,num),map(float,den))

    y,t = matlab.step([tf],*args,**kwargs)

    plt.plot(t,y)

    plt.title("Step Response")

    plt.grid()

    plt.xlabel("time (s)")

    plt.ylabel("y(t)")

    info ="OS: %f%s"%(round((y.max()/y[-1]-1)*100,2),'%')

    try:

        i10 = next(i for i in range(0,len(y)-1) if y[i]>=y[-1]*.10)

        Tr = round(t[next(i for i in range(i10,len(y)-1) if y[i]>=y[-1]*.90)]-t[i10],2)

    except StopIteration:

        Tr = "unknown"

    try:

        Ts = round(t[next(len(y)-i for i in range(2,len(y)-1) if abs(y[-i]/y[-1])>1.02)]-t[0],2)

    except StopIteration:

        Ts = "unknown"

        

    info += "\nTr: %s"%(Tr)

    info +="\nTs: %s"%(Ts)

    print info

    plt.legend([info],loc=4)

    plt.show()