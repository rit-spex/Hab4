/**
 * Rolling.cpp
 * Determines if more rcent values are lower than older ones.
 * Uses rolling sums, to suppress any noise
*/

class Rolling {
public:
    void addValue(int val) {
        double old1 = recent[i1];
        recent[i1] = val;
        i1 = (i1+1) % LEN;
        recentVal -= old1;
        recentVal += val;

        double old2 = older[i2];
        older[i2] = old1;
        i2 = (i2+1) % LEN;
        olderVal -= old2;
        olderVal += old1;
    }

    int isDecreasing() {
        if(recentVal < olderVal) {
            return 1;
        } else {
            return 0;
        }
    }

private:
    static const int LEN = 5;
    int i1 = 0;
    int i2 = 0;
    double recentVal = 0;
    double olderVal = 0;
    double recent[LEN] = { 0 };
    double older[LEN] = { 0 };
};

