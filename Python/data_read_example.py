data=io.read_array('fourier_hw1.txt',separator='\t')#read data from file (tab-delimited)
#Note: Python always counts from 0 for indices
data=data[1:,:]#eliminate first row of data (row 0)
t=data[:,0]#t is the first column (column 0)
y=data[:,1]#y is the second column (column 1)
plot(t,y)