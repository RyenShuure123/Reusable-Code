#include "app_bmi270.h"

int main(void)
{
    App_BMI270_Init();

    while (1) {
        App_BMI270_RunOnce();
    }
}
