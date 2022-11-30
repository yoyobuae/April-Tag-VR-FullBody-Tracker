#include <stdlib.h>
#include "tagCustom28h9.h"

static uint64_t codedata[137] = {
   0x000000000ba48a3fUL,
   0x0000000000c295c9UL,
   0x000000000b519b8eUL,
   0x0000000005e0a153UL,
   0x000000000afeacddUL,
   0x00000000058db2a2UL,
   0x00000000053ac3f1UL,
   0x000000000fc9c9b6UL,
   0x000000000a05e0caUL,
   0x000000000494e68fUL,
   0x000000000441f7deUL,
   0x000000000ed0fda3UL,
   0x00000000090d14b7UL,
   0x000000000d8542dfUL,
   0x00000000071b7c91UL,
   0x0000000006c88de0UL,
   0x000000000be6996aUL,
   0x000000000b93aab9UL,
   0x000000000a9adea6UL,
   0x00000000094f23e2UL,
   0x00000000038b3af6UL,
   0x0000000008a94680UL,
   0x00000000089866faUL,
   0x000000000db67284UL,
   0x000000000c6ab7c0UL,
   0x000000000e9e5f11UL,
   0x0000000003bc6a9bUL,
   0x000000000177e3c4UL,
   0x000000000c06e989UL,
   0x00000000096f7401UL,
   0x000000000305adb3UL,
   0x000000000d94b378UL,
   0x00000000072aed2aUL,
   0x00000000001b497aUL,
   0x0000000003ed9a40UL,
   0x000000000e29b154UL,
   0x000000000482981cUL,
   0x000000000d6214e6UL,
   0x000000000c6948d3UL,
   0x0000000008861887UL,
   0x000000000c58694dUL,
   0x000000000a13e276UL,
   0x000000000ced6729UL,
   0x000000000b2cfea8UL,
   0x000000000749ce5cUL,
   0x0000000001c7f49bUL,
   0x00000000062f433dUL,
   0x0000000006b36193UL,
   0x000000000949682eUL,
   0x000000000e148507UL,
   0x0000000000687c77UL,
   0x00000000061b85ddUL,
   0x0000000004082eabUL,
   0x000000000d392b39UL,
   0x000000000f295472UL,
   0x0000000003f3025eUL,
   0x000000000b650558UL,
   0x00000000035b26a8UL,
   0x0000000009e25f3bUL,
   0x0000000002c1dc05UL,
   0x00000000097d2217UL,
   0x000000000bcb5dd3UL,
   0x000000000cf3ea9eUL,
   0x000000000282571eUL,
   0x00000000074d73f7UL,
   0x0000000006954822UL,
   0x00000000035a7f7cUL,
   0x0000000004722cc1UL,
   0x0000000002427ea1UL,
   0x0000000001e6f662UL,
   0x000000000e1ceb94UL,
   0x000000000187215cUL,
   0x000000000d8ec4c3UL,
   0x0000000006bce377UL,
   0x000000000527eee7UL,
   0x0000000006519709UL,
   0x00000000095a88deUL,
   0x000000000debdc84UL,
   0x000000000bbabf77UL,
   0x0000000002c8f43aUL,
   0x00000000066b30b2UL,
   0x0000000005d13ee3UL,
   0x0000000006bd24a1UL,
   0x000000000c231b9fUL,
   0x000000000ec83f3dUL,
   0x0000000004156453UL,
   0x00000000026ee911UL,
   0x000000000f6c47b0UL,
   0x000000000821184aUL,
   0x000000000df160f4UL,
   0x000000000da7a091UL,
   0x00000000050281baUL,
   0x0000000002c93f4aUL,
   0x000000000e0d2d08UL,
   0x00000000096bc19dUL,
   0x00000000030bd0c9UL,
   0x000000000e3af83cUL,
   0x00000000007232bcUL,
   0x000000000c9ec25cUL,
   0x000000000a76b308UL,
   0x0000000005b23fdcUL,
   0x000000000b9781d2UL,
   0x0000000009ebfbeeUL,
   0x0000000008ccf107UL,
   0x0000000000539574UL,
   0x000000000260e2ffUL,
   0x000000000f78a025UL,
   0x000000000219b3e3UL,
   0x00000000050146e8UL,
   0x000000000f62c2a3UL,
   0x000000000e12ef79UL,
   0x000000000787dfaaUL,
   0x00000000068ffd4dUL,
   0x000000000921eb82UL,
   0x000000000e5964b7UL,
   0x000000000acc0699UL,
   0x000000000cc91787UL,
   0x000000000ead0f57UL,
   0x0000000009d2368fUL,
   0x0000000000bc9e90UL,
   0x000000000717ba60UL,
   0x000000000ce46a15UL,
   0x000000000746aaaeUL,
   0x00000000015831e4UL,
   0x000000000f86412eUL,
   0x000000000d926851UL,
   0x000000000d6d868eUL,
   0x0000000008c64a68UL,
   0x0000000001a1cc7eUL,
   0x0000000001df4251UL,
   0x0000000001daedfbUL,
   0x0000000008082196UL,
   0x0000000009c99979UL,
   0x0000000009063d53UL,
   0x000000000e1623bfUL,
   0x000000000318c11eUL,
   0x0000000008cb6d14UL,
};
apriltag_family_t *tagCustom28h9_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tagCustom28h9");
   tf->h = 9;
   tf->ncodes = 137;
   tf->codes = codedata;
   tf->nbits = 28;
   tf->bit_x = calloc(28, sizeof(uint32_t));
   tf->bit_y = calloc(28, sizeof(uint32_t));
   tf->bit_x[0] = -1;
   tf->bit_y[0] = -3;
   tf->bit_x[1] = 0;
   tf->bit_y[1] = -3;
   tf->bit_x[2] = 1;
   tf->bit_y[2] = -3;
   tf->bit_x[3] = -2;
   tf->bit_y[3] = -2;
   tf->bit_x[4] = -1;
   tf->bit_y[4] = -2;
   tf->bit_x[5] = 0;
   tf->bit_y[5] = -2;
   tf->bit_x[6] = 1;
   tf->bit_y[6] = -2;
   tf->bit_x[7] = 3;
   tf->bit_y[7] = -1;
   tf->bit_x[8] = 3;
   tf->bit_y[8] = 0;
   tf->bit_x[9] = 3;
   tf->bit_y[9] = 1;
   tf->bit_x[10] = 2;
   tf->bit_y[10] = -2;
   tf->bit_x[11] = 2;
   tf->bit_y[11] = -1;
   tf->bit_x[12] = 2;
   tf->bit_y[12] = 0;
   tf->bit_x[13] = 2;
   tf->bit_y[13] = 1;
   tf->bit_x[14] = 1;
   tf->bit_y[14] = 3;
   tf->bit_x[15] = 0;
   tf->bit_y[15] = 3;
   tf->bit_x[16] = -1;
   tf->bit_y[16] = 3;
   tf->bit_x[17] = 2;
   tf->bit_y[17] = 2;
   tf->bit_x[18] = 1;
   tf->bit_y[18] = 2;
   tf->bit_x[19] = 0;
   tf->bit_y[19] = 2;
   tf->bit_x[20] = -1;
   tf->bit_y[20] = 2;
   tf->bit_x[21] = -3;
   tf->bit_y[21] = 1;
   tf->bit_x[22] = -3;
   tf->bit_y[22] = 0;
   tf->bit_x[23] = -3;
   tf->bit_y[23] = -1;
   tf->bit_x[24] = -2;
   tf->bit_y[24] = 2;
   tf->bit_x[25] = -2;
   tf->bit_y[25] = 1;
   tf->bit_x[26] = -2;
   tf->bit_y[26] = 0;
   tf->bit_x[27] = -2;
   tf->bit_y[27] = -1;
   tf->width_at_border = 1;
   tf->total_width = 7;
   tf->reversed_border = true;
   return tf;
}

void tagCustom28h9_destroy(apriltag_family_t *tf)
{
   free(tf->bit_x);
   free(tf->bit_y);
   free(tf->name);
   free(tf);
}
