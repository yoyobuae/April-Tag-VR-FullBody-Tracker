#include <stdlib.h>
#include "tagCustom28h9.h"

static uint64_t codedata[133] = {
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
   0x0000000006c88de0UL,
   0x000000000be6996aUL,
   0x000000000b93aab9UL,
   0x000000000a9adea6UL,
   0x00000000094f23e2UL,
   0x00000000038b3af6UL,
   0x0000000008a94680UL,
   0x00000000093e445cUL,
   0x00000000089866faUL,
   0x000000000db67284UL,
   0x00000000022ea0acUL,
   0x000000000e9e5f11UL,
   0x0000000003bc6a9bUL,
   0x000000000177e3c4UL,
   0x000000000c06e989UL,
   0x00000000096f7401UL,
   0x000000000305adb3UL,
   0x000000000d94b378UL,
   0x00000000072aed2aUL,
   0x0000000003ed9a40UL,
   0x000000000e29b154UL,
   0x00000000057b642fUL,
   0x000000000482981cUL,
   0x000000000e5ae0f9UL,
   0x000000000c58694dUL,
   0x000000000a13e276UL,
   0x000000000ced6729UL,
   0x000000000b2cfea8UL,
   0x000000000749ce5cUL,
   0x0000000001c7f49bUL,
   0x00000000062f433dUL,
   0x000000000a5482b4UL,
   0x0000000006b36193UL,
   0x000000000de35562UL,
   0x0000000000158dc6UL,
   0x000000000d7e183eUL,
   0x00000000060aa657UL,
   0x000000000a82d47fUL,
   0x0000000000e9bcf3UL,
   0x000000000f49a491UL,
   0x0000000001197dabUL,
   0x000000000073a049UL,
   0x0000000005a11c6cUL,
   0x000000000aacd983UL,
   0x000000000b62277eUL,
   0x0000000006538c8dUL,
   0x00000000097ffff1UL,
   0x00000000097d2217UL,
   0x00000000012e5662UL,
   0x0000000004282b34UL,
   0x0000000009355738UL,
   0x000000000bcb5dd3UL,
   0x000000000cf3ea9eUL,
   0x00000000074d73f7UL,
   0x000000000dd78a64UL,
   0x000000000b5d8721UL,
   0x0000000007652a88UL,
   0x0000000004b15e12UL,
   0x000000000dafbc0eUL,
   0x000000000cc77bebUL,
   0x000000000634785aUL,
   0x000000000527eee7UL,
   0x000000000ec94c6bUL,
   0x000000000624b42bUL,
   0x000000000156124cUL,
   0x000000000786d728UL,
   0x0000000002426c35UL,
   0x000000000742b1e4UL,
   0x0000000005916666UL,
   0x000000000e67f60cUL,
   0x0000000001c1c079UL,
   0x000000000a89faddUL,
   0x000000000bdc3916UL,
   0x000000000230d317UL,
   0x000000000db85159UL,
   0x000000000489df0fUL,
   0x00000000085eba19UL,
   0x000000000fd08a12UL,
   0x0000000005bf3fc3UL,
   0x000000000c556258UL,
   0x0000000001f4ae5eUL,
   0x000000000ab2061aUL,
   0x00000000072151b4UL,
   0x000000000e2d5df2UL,
   0x000000000ac0460dUL,
   0x00000000059fd0d4UL,
   0x0000000001cdd504UL,
   0x00000000024325aaUL,
   0x00000000026899daUL,
   0x00000000024efbeeUL,
   0x0000000002c19935UL,
   0x000000000d354871UL,
   0x0000000009dedb4fUL,
   0x0000000002c754c6UL,
   0x000000000cb199e0UL,
   0x000000000fb24b60UL,
   0x000000000a068487UL,
   0x000000000735d083UL,
   0x0000000003a6e443UL,
   0x000000000cd201a5UL,
   0x000000000685f8acUL,
   0x000000000d882dcaUL,
   0x000000000b341e05UL,
   0x0000000000ba0a66UL,
   0x000000000ee53d13UL,
   0x00000000035a6843UL,
   0x000000000f1f7748UL,
   0x000000000c7bb2a5UL,
   0x00000000076b4ae2UL,
   0x000000000c9f1cfbUL,
   0x000000000e0c46ecUL,
   0x000000000fba9d57UL,
   0x000000000239e588UL,
   0x00000000082c729eUL,
   0x0000000004f682fdUL,
   0x0000000007ecd0baUL,
   0x0000000000e7fd6dUL,
   0x000000000f12813cUL,
   0x0000000000abf984UL,
};
apriltag_family_t *tagCustom28h9_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tagCustom28h9");
   tf->h = 9;
   tf->ncodes = 133;
   tf->codes = codedata;
   tf->nbits = 28;
   tf->bit_x = calloc(28, sizeof(uint32_t));
   tf->bit_y = calloc(28, sizeof(uint32_t));
   tf->bit_x[0] = 0;
   tf->bit_y[0] = -3;
   tf->bit_x[1] = 1;
   tf->bit_y[1] = -3;
   tf->bit_x[2] = -2;
   tf->bit_y[2] = -2;
   tf->bit_x[3] = -1;
   tf->bit_y[3] = -2;
   tf->bit_x[4] = 0;
   tf->bit_y[4] = -2;
   tf->bit_x[5] = 1;
   tf->bit_y[5] = -2;
   tf->bit_x[6] = 2;
   tf->bit_y[6] = -2;
   tf->bit_x[7] = 4;
   tf->bit_y[7] = 0;
   tf->bit_x[8] = 4;
   tf->bit_y[8] = 1;
   tf->bit_x[9] = 3;
   tf->bit_y[9] = -2;
   tf->bit_x[10] = 3;
   tf->bit_y[10] = -1;
   tf->bit_x[11] = 3;
   tf->bit_y[11] = 0;
   tf->bit_x[12] = 3;
   tf->bit_y[12] = 1;
   tf->bit_x[13] = 3;
   tf->bit_y[13] = 2;
   tf->bit_x[14] = 1;
   tf->bit_y[14] = 4;
   tf->bit_x[15] = 0;
   tf->bit_y[15] = 4;
   tf->bit_x[16] = 3;
   tf->bit_y[16] = 3;
   tf->bit_x[17] = 2;
   tf->bit_y[17] = 3;
   tf->bit_x[18] = 1;
   tf->bit_y[18] = 3;
   tf->bit_x[19] = 0;
   tf->bit_y[19] = 3;
   tf->bit_x[20] = -1;
   tf->bit_y[20] = 3;
   tf->bit_x[21] = -3;
   tf->bit_y[21] = 1;
   tf->bit_x[22] = -3;
   tf->bit_y[22] = 0;
   tf->bit_x[23] = -2;
   tf->bit_y[23] = 3;
   tf->bit_x[24] = -2;
   tf->bit_y[24] = 2;
   tf->bit_x[25] = -2;
   tf->bit_y[25] = 1;
   tf->bit_x[26] = -2;
   tf->bit_y[26] = 0;
   tf->bit_x[27] = -2;
   tf->bit_y[27] = -1;
   tf->width_at_border = 2;
   tf->total_width = 8;
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
