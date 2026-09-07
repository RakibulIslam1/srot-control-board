// Host-side proof that suppressing KILL removes a real ambiguity.
// The emit condition is COPIED VERBATIM from the patch.
#include <cstdio>
#include <cassert>
#include <vector>
#include <string>

struct Snap { bool kill; bool pm2_present; };

// What a consumer sees: the value, or nothing at all.
struct Seen { bool sent; float value; };

static Seen emit(const Snap& s) {
    Seen out{false, 0.0f};
    if (s.pm2_present) { out.sent = true; out.value = s.kill ? 1.0f : 0.0f; }   // verbatim
    return out;
}

// The three physical situations that matter.
static const Snap LIVE   {false, true };   // 2nd board talking, power live
static const Snap CUT    {true,  true };   // 2nd board talking, power CUT
static const Snap NOLINK {false, false};   // no 2nd board -- espnow_link forces kill=false

int main() {
    // 1. With suppression, every distinct physical state is distinguishable.
    Seen live = emit(LIVE), cut = emit(CUT), nolink = emit(NOLINK);
    assert(live.sent && live.value == 0.0f);
    assert(cut.sent  && cut.value  == 1.0f);
    assert(!nolink.sent);                       // absence, not a confident zero
    assert(!(live.sent == nolink.sent && live.value == nolink.value));

    // 2. THE BUG THIS REMOVES: without the gate, "live" and "no link" are the
    //    SAME BYTES on the wire. Reproduced here so the claim is not rhetorical.
    auto ungated = [](const Snap& s) { return Seen{true, s.kill ? 1.0f : 0.0f}; };
    Seen u_live = ungated(LIVE), u_nolink = ungated(NOLINK);
    assert(u_live.sent == u_nolink.sent && u_live.value == u_nolink.value);

    // 3. Suppression must never hide a REAL cut -- that would trade one silent
    //    failure for a worse one.
    assert(emit(CUT).sent && emit(CUT).value == 1.0f);

    printf("KILL suppression: 3 physical states, 3 distinguishable outcomes.\n");
    printf("Ungated: 'live' and 'no link' are byte-identical (the bug).\n");
    return 0;
}
