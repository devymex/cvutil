all: libcvutil.a

libcvutil.a: cvutil.o
	rm $@
	ar rcs $@ *.o
	rm *.o

cvutil.o: cvutil.cpp cvutil.hpp
	g++ -std=c++11 -c -o $@ $<

clean:
	rm -f *.o *.a

