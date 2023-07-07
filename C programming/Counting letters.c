#include <stdio.h>
/*This program gets a string of 30 characters or less.
If an asterisk is encountered, the string is truncated.*/
/*#define MAXLEN 30
int main(void) {
	int count;
	char buffer[MAXLEN + 1];
	puts("Enter text, enter an asterisk to exit.");
	while (1) {
		gets(buffer);
		for (count = 0; count < (MAXLEN + 1); count++) {
			if (buffer[count] == '*') {
				putchar('\n');
				break;
			}
			else {
				putchar(buffer[count]);
			}
		}
	}
	return 0;
}*/

/*This program redirects a file to the printer one
character at a time*/
/*int main(void) {
	int ctr;
	char buf[30];
	puts("Enter string:");
	gets(buf);
	for (ctr = 0; ctr < 30; ctr++) {
		printf("%c\n", buf[ctr]);
	}
}*/

/*This program counts the number of times each letter
occurs in the entered string*/
/* Considering characters from 'a' to 'z' only and ignoring others. */
int main() {
	char string[100];
	int c = 0, d = 0, lowercase_count[26] = { 0 };
	int x, y, uppercase_count[26] = { 0 };

	printf("Enter a string (lower case only)\n");
	gets(string);

	while (string[c] != '\0') {
		if (string[c] >= 'a' && string[c] <= 'z') {
			x = string[c] - 'a';
			lowercase_count[x]++;
		}
		c++;
	}
	while (string[d] != '\0') {
		if (string[d] >= 'A' && string[d] <= 'Z') {
			y = string[d] - 'A';
			uppercase_count[y]++;
		}
		d++;
	}
	for (c = 0; c < 26; c++)
		printf("%c occurs %d times in the string.\n", c + 'a', lowercase_count[c]);
	for (d = 0; d < 26; d++)
		printf("%c occurs %d times in the string.\n", d + 'A', uppercase_count[d]);
	return 0;
}
	