///shell sort///
void shell_sort(int A[], int size) {
  int i, j, increment, temp;
  increment = size / 2;

  while (increment > 0) {
    for (i = increment; i < size; i++) {
      j = i;
      temp = A[i];
      while ((j >= increment) && (A[j-increment] > temp)) {
        A[j] = A[j - increment];
        j = j - increment;
      }
      A[j] = temp;
    }

    if (increment == 2)
       increment = 1;
    else
       increment = (int) (increment / 2.2);
  }
}
