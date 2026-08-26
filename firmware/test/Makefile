CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -g

test: test_core
	./test_core

test_core: test_core.cpp ../src/haptics_core.h
	$(CXX) $(CXXFLAGS) -o $@ test_core.cpp

clean:
	rm -f test_core

.PHONY: test clean
