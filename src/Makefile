CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv` -lcblas
INCLUDE = -I /usr/local/lib64
CC = g++ -std=c++11
% : %.cpp
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)
