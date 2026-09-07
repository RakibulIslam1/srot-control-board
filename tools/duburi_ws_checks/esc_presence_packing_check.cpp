// Host-side proof of the ESC-presence packing in mav_stream.cpp.
// The two loop bodies below are COPIED VERBATIM from the patch.
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cassert>
static const int NUM_THRUSTERS = 8;

static void pack(uint8_t esc_present, const int16_t rpm[8],
                 uint16_t cnt1[4], uint16_t cnt2[4]) {
    uint16_t erpm[4] = {0};
    struct { const int16_t* rpm; uint8_t esc_present; } s{rpm, esc_present};

    for (int i = 0; i < 4; ++i) {                       // block 1..4  (verbatim)
        erpm[i] = (uint16_t)abs((int)s.rpm[i]);
        cnt1[i] = (uint16_t)((s.esc_present >> i) & 1);
    }
    for (int i = 0; i < 4; ++i) {                       // block 5..8  (verbatim)
        erpm[i] = (4 + i < NUM_THRUSTERS) ? (uint16_t)abs((int)s.rpm[4 + i]) : 0;
        cnt2[i] = (4 + i < NUM_THRUSTERS)
                  ? (uint16_t)((s.esc_present >> (4 + i)) & 1) : 0;
    }
}

int main() {
    int16_t rpm[8] = {0,0,0,0,0,0,0,0};
    uint16_t a[4], b[4];

    // 1. nothing attached -> every presence bit 0, which is the case that is
    //    currently INDISTINGUISHABLE from a healthy idle hull.
    pack(0x00, rpm, a, b);
    for (int i = 0; i < 4; ++i) { assert(a[i] == 0); assert(b[i] == 0); }

    // 2. all eight attached
    pack(0xFF, rpm, a, b);
    for (int i = 0; i < 4; ++i) { assert(a[i] == 1); assert(b[i] == 1); }

    // 3. THE ONE THAT MATTERS: bit i must land in slot i of the RIGHT block.
    //    Thruster 6 is bit 5 -> block 5..8, index 1.
    pack(1u << 5, rpm, a, b);
    for (int i = 0; i < 4; ++i) assert(a[i] == 0);
    assert(b[0] == 0 && b[1] == 1 && b[2] == 0 && b[3] == 0);

    //    Thruster 1 is bit 0 -> block 1..4, index 0.
    pack(1u << 0, rpm, a, b);
    assert(a[0] == 1 && a[1] == 0 && a[2] == 0 && a[3] == 0);
    for (int i = 0; i < 4; ++i) assert(b[i] == 0);

    //    Thruster 8 is bit 7 -> block 5..8, index 3 (the >> 4+i off-by-one trap).
    pack(1u << 7, rpm, a, b);
    assert(b[3] == 1 && b[0] == 0 && b[1] == 0 && b[2] == 0);

    // 4. presence is INDEPENDENT of rpm: an attached thruster commanded to zero
    //    must still report present. This is the rule the PR says not to break.
    int16_t still[8] = {0,0,0,0,0,0,0,0};
    pack(0xFF, still, a, b);
    for (int i = 0; i < 4; ++i) { assert(a[i] == 1); assert(b[i] == 1); }

    // 5. a REVERSED thruster is still present (sign must not leak into presence)
    int16_t rev[8] = {-1200,-1200,1200,1200,0,0,0,0};
    pack(0x0F, rev, a, b);
    for (int i = 0; i < 4; ++i) assert(a[i] == 1);

    printf("all presence-packing assertions PASSED\n");
    return 0;
}
