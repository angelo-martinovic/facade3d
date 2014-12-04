
cflags = `pkg-config --cflags opencv` 
ldflags = `pkg-config --libs opencv` -lstdc++
sources = LineUtils.cpp LineDetector.cpp
objects = LineUtils.o LineDetector.o


detectLines: detectLines.cpp $(objects)
	g++ $(cflags) $(ldflags) -o $@ $(objects) $<

%.o: %.cpp
	g++ -g -c $(cflags) -o $@ $<

clean:
	rm -f *.o

