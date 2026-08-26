CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -g

test: test_core
	./test_core

test_core: test_core.cpp ../pedal_haptics/src/haptics_core.h
	$(CXX) $(CXXFLAGS) -o $@ test_core.cpp

clean:
	rm -rf test_core test_core.dSYM

.PHONY: test clean
