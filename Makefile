
main: main.o System.o Coupled.o Controller.o
	g++ main.o System.o Coupled.o Controller.o -o main

System.hpp.gch: System.hpp
	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c System.hpp

System.o: System.cpp System.hpp.gch
	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c System.cpp

Controller.hpp.gch: Controller.hpp
	g++ -c Controller.hpp

Controller.o: Controller.cpp Controller.hpp.gch 
	g++ -c Controller.cpp

Coupled.hpp.gch: Coupled.hpp System.hpp.gch Controller.hpp.gch
	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c Coupled.hpp

Coupled.o: Coupled.cpp Coupled.hpp.gch
	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c Coupled.cpp

main.o: main.cpp Coupled.hpp.gch System.hpp.gch
	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c main.cpp

clean:
	rm *.o *.hpp.gch main



########################################################
########################################################


# main: main.o Pendulum.hpp.gch Motor.hpp.gch System.hpp.gch Coupled.hpp.gch Controller.hpp.gch
# 	g++ main.o -o main

# System.hpp.gch: System.hpp constants.h
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c System.hpp

# Pendulum.hpp.gch: Pendulum.hpp System.hpp.gch constants.h
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c Pendulum.hpp

# Motor.hpp.gch: Motor.hpp System.hpp.gch constants.h
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c Motor.hpp

# Controller.hpp.gch: Controller.hpp
# 	g++ -c Controller.hpp

# Coupled.hpp.gch: Coupled.hpp System.hpp.gch Pendulum.hpp.gch Motor.hpp.gch Controller.hpp.gch constants.h
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c Coupled.hpp

# main.o: main.cpp Coupled.hpp.gch
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c main.cpp

# clean:
# 	rm *.o *.hpp.gch main

########################################################
########################################################

# output: main.o Pendulum.h.gch Motor.h.gch System.h.gch Coupled.h.gch Controller.h.gch
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen main.o Pendulum.hpp Motor.hpp System.hpp Coupled.hpp Controller.hpp

# System.h.gch: System.hpp constants.h
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c System.hpp

# Pendulum.h.gch: Pendulum.hpp System.h.gch constants.h
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c Pendulum.hpp

# Motor.h.gch: Motor.hpp System.h.gch constants.h
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c Motor.hpp

# Controller.h.gch: Controller.hpp
# 	g++ -c Controller.hpp

# Coupled.h.gch: Coupled.hpp System.hpp.gch Pendulum.hpp.gch Motor.hpp.gch Controller.hpp.gch constants.h
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c Coupled.hpp

# main.o: main.cpp Coupled.h.gch
# 	g++ -I /Users/dinsogna/Documents/RoMeLa/eigen -c main.cpp

# clean:
# 	rm *.o *.h.gch a.out














