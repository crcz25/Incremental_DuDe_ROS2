/////////////////////////////////////////////////////////////////////
//
//  Defines.h
//
//  General Description
//  This an example of how the includes should look like in order to have
//  compatibility with different STLs Created
//      Gabriel Tanase
//
/////////////////////////////////////////////////////////////////////

// include standard headers
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/*
#ifndef  _STL_PORT

#if defined(__KCC)
#include <iostream.h>
#include <fstream.h>
#include <iomanip.h>
#include <algorithm>
#include <list>
#include <vector>
#include <deque>
#include <stack>
#include <map>
#include <iterator>

#else
#if defined(sun) || defined(__sgi) || defined(__linux__) || defined(_WIN32)
#include <iostream.h>
#include <fstream.h>
#include <iomanip.h>
#include <algo.h>
#include <list.h>
#include <vector.h>
#include <deque.h>
#include <stack.h>
#include <map.h>
#include <iterator.h>
#endif
#endif

#if defined(hppa) && defined(__cplusplus)   //c++ in parasol
#include <iostream.h>
#include <fstream.h>
#include <iomanip.h>
#include <algorithm>
#include <list>
#include <vector>
#include <deque>
#include <stack>
#include <map>
#include <iterator>
#endif

//aCC in parasol
#ifdef __HP_aCC
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <list>
#include <vector>
#include <deque>
#include <stack>
#include <map>
#include <iterator>
#endif
using namespace std;

#ifdef _DARWIN
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <list>
#include <vector>
#include <deque>
#include <stack>
#include <map>
#include <iterator>
#endif

//STL_PORT
#else
*/
#include <algorithm>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <list>
#include <map>
#include <ostream>
#include <queue>
#include <set>
#include <stack>
#include <vector>
// #include <hash_map>
// using namespace _STLP_STD;
using namespace std;
// #endif

#ifndef OK
#define OK 0
#endif

#ifndef ERROR
#define ERROR -1
#endif
