/*This program sorts 10 user defined double variables into ascending
or descending order.*/
#include <stdio.h>

#define MAX 10

void sort_and_print(double p[], int n, int sort_type);

double array[MAX];

int main(void) {
	int i;
	int sort_type;
	while (1) {
		puts("Please enter zero to stop providing values.");
		for (i = 0; i < 10; i++) { /*Collect values for 10-element array*/
			printf("Enter value for array element %d: ", i + 1);
			scanf_s("%lf", &array[i]);
			if (array[i] == 0)
				break;  /*Entering zero without entering a value exits the for loop*/
		}				
		do {
			puts("Enter 0 for descending order, 1 for ascending order: ");
			scanf_s("%d", &sort_type);
		} while (sort_type != 0 && sort_type != 1);
		sort_and_print(array, MAX, sort_type);
	}
	return 0;
}

void sort_and_print(double p[], int n, int sort_type) {
	int ctr, y, z, b;
	double x;
	if (sort_type == 0) {
		for (z = 1; z < n; z++) {
			for (ctr = 0; ctr < n - 1; ctr++) {
				if ((p[ctr + 1] - p[ctr]) > 0) {
					x = p[ctr];
					p[ctr] = p[ctr + 1];
					p[ctr + 1] = x;
				}
			}
		}
	}
	if (sort_type == 1) {
		for (b = 1; b < n; b++) {
			for (ctr = 0; ctr < n - 1; ctr++) {
				if ((p[ctr] - p[ctr + 1]) > 0) {
					x = p[ctr];
					p[ctr] = p[ctr + 1];
					p[ctr + 1] = x;
				}
			}
		}
	}
	for (y = 0; y < n; y++) {
		printf("%lf\n", p[y]);
	}
}
