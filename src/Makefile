CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv` 
CC = g++ -std=c++11
% : %.cpp
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)
