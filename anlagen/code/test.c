    void main() {
        const char *first_string = "abc"; //Definieren eines Strings
        const char *second_string = "abc";
        int result = mx_strcmp(first_string, second_string);

        if (result != 0) {
            printf("Vergleich schlug fehl!\n");
        }
    }