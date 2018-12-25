all: libcvutil.a

libcvutil.a: cvutil.o dir.o
	rm $@
	ar rcs $@ *.o
	rm *.o

cvutil.o: cvutil.cpp cvutil.hpp
	g++ -std=c++11 -c -o $@ $<
	
dir.o:
	ar -x /usr/lib/gcc/x86_64-linux-gnu/7/libstdc++fs.a

clean:
	rm -f *.o *.a

