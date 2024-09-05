CC = gcc
CFLAGS = -g -Wall -Wextra -Wpedantic --std=c17

# Target
generate: generate.o
	$(CC) $(CFLAGS) $^ -o $@ -lm

# Target 
knn: knn.o
	$(CC) $(CFLAGS) $^ -o $@ -lm

knn.o: knn.c
	$(CC) $(CFLAGS) -c $< -o $@ -lm

generate.o: generate.c
	$(CC) $(CFLAGS) -c $< -o $@ -lm

clean:
	rm *.o knn generate