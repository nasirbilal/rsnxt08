typedef struct
{
  char x; //1
  char y; //1
} struct1;

typedef struct
{
	struct1 Test[2];
} struct2;

struct2 TESTER;
struct1 put;

void func(struct1 put)
{
  TESTER.Test[0] = put;
}


task main()
{
	memset(TESTER,0,4);

  //put.x = 1;
 // put.y = 2;
  func(put);

}
