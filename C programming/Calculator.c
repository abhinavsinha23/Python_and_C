#include <stdio.h>
#include <stdlib.h>
/*This program works like a calculator*/
float a, b;
int choice;
int get_choice(void);
int main(void) {
	printf("Enter first number:\n");
	scanf_s("%f", &a);

	printf("\nEnter second number:\n");
	scanf_s("%f", &b);
	choice = get_choice();

	switch (choice) {
		case 1: {
			printf("%f + %f = %f", a, b, (a+b));
			break;
		}
		case 2: {
			printf("%f - %f = %f", a, b, (a-b));
			break;
		}
		case 3: {
			printf("%f * %f = %f", a, b, (a*b));
			break;
		}
		case 4: {
			printf("%f / %f = %f", a, b, (a/b));
			break;
		}
		case 5: {
			printf("Exiting the program...");
			exit(0);
		}
	}
	return 0;
}

int get_choice(void) {
	int c;
	printf("Which mathematical operation do you need?\n");
	printf("Press 1 for addition\n");
	printf("Press 2 for subtraction\n");
	printf("Press 3 for multiplication\n");
	printf("Press 4 for division\n");
	printf("Press 5 to exit\n");
	scanf_s("%d", &c);

	while (c < 1 || c > 5) {
		printf("You entered an invalid choice. Please try again.\n");
		scanf_s("%d", &c);
	}
	return c;
}