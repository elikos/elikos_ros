require(RcppCNPy)

data <- npyLoad(filename = "result.npy")


plot(data[1,], type="l", ylim=c(min(data), max(data)), col="red")
lines(data[2,], col = "dark green")
lines(data[3,], col="blue")
#lines(data[4,])

