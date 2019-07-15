#include "avionics_sim/Bilinear_interp.hpp"

#include <gtest/gtest.h>
#include <iostream> 
#include <vector>
#include <string>

namespace 
{
    class BilinearInterp_UnitTest:public ::testing::Test
    {
        protected:
            std::string xValsString =  "0.0,596.92,1193.6,1790.4,2387.1,2983.8,3580.5,4177.2,4774.0,5370.7,5967.4,6564.1,7160.8,7757.6,8354.3,8951.0,8951.0;0.0,606.61,1212.8,1819.0,2425.2,3031.4,3637.6,4243.8,4850.1,5456.3,6062.5,6668.7,7274.9,7881.1,8487.3,9093.5,9093.5;0.0,2265.4,2719.2,3173.0,3626.7,4080.5,4534.3,4988.1,5441.9,5895.6,6349.4,6803.2,7257.0,7710.8,8164.5,8618.3,9072.1;0.0,3115.6,3511.0,3906.4,4301.8,4697.1,5092.5,5487.9,5883.3,6278.7,6674.1,7069.5,7464.9,7860.2,8255.6,8651.0,9046.4;0.0,3915.8,4256.5,4597.1,4937.8,5278.4,5619.1,5959.8,6300.4,6641.1,6981.7,7322.4,7663.1,8003.7,8344.4,8685.0,9025.7;0.0,4854.5,5134.3,5414.1,5693.9,5973.6,6253.4,6533.2,6813.0,7092.8,7372.6,7652.4,7932.2,8211.9,8491.7,8771.5,9051.3;0.0,5685.4,5915.7,6146.1,6376.4,6606.8,6837.1,7067.5,7297.8,7528.2,7758.5,7988.9,8219.2,8449.6,8679.9,8910.3,9140.6;";
            std::string yValsString = "0,20.0,30.0,40.0,50.0,60.0,70.0";
            std::string zValsString = "0.000,0.0365,0.0945,0.1854,0.3384,0.5211,0.7346,1.0090,1.3199,1.6988,2.0982,2.5728,3.0525,3.5715,4.1525,4.8130,4.8130;-0.0549,-0.068,-0.064,-0.009,0.1148,0.3348,0.6418,0.9495,1.3390,1.7405,2.1760,2.7216,3.2964,3.8947,4.5780,5.3292,5.3292;-0.099,-0.099,-0.007,0.1346,0.3231,0.5814,0.9056,1.2012,1.5351,1.8781,2.2385,2.6773,3.0722,3.5498,4.0396,4.5546,5.1445;-0.1448,-0.144,-0.021,0.1391,0.3421,0.5805,0.9337,1.237,1.5905,1.875,2.2892,2.6842,3.0426,3.4662,3.9501,4.4186,4.8634;-0.2151,-0.215,-0.059,0.1061,0.3085,0.5413,0.7892,1.1035,1.5326,1.8159,2.1519,2.5047,2.9243,3.3057,3.6779,4.0637,4.5053;-0.3076,-0.307,-0.149,0.0157,0.2059,0.3860,0.6456,0.9408,1.2252,1.5840,1.9012,2.2649,2.5861,2.8896,3.2757,3.6138,3.9328;-0.435,-0.435,-0.303,-0.139,0.0614,0.2475,0.4780,0.6999,0.9397,1.2130,1.5239,1.8279,2.1078,2.4171,2.7365,3.0420,3.3587;";

    };

    TEST_F(BilinearInterp_UnitTest, get1DLUTelementsFromString)
    {
        std::string badVals1 = "0,20.0,30.0,,50.0,60.0,70.0";
        std::string badVals2 = "";
        std::string badVals3 = "0,20.0,30.0,40.0,50.0,60.0,abc";

        std::vector<float> outVect; 

        // Expect success == false for string with null element. 
        EXPECT_FALSE(avionics_sim::Bilinear_interp::get1DLUTelementsFromString(badVals1, &outVect));

        // Expect success == false for null string
        EXPECT_FALSE(avionics_sim::Bilinear_interp::get1DLUTelementsFromString(badVals2, &outVect));

        // Expect success == false for bad last element
        EXPECT_FALSE(avionics_sim::Bilinear_interp::get1DLUTelementsFromString(badVals3, &outVect));

        // Expect successful parse with proper values.        
        EXPECT_TRUE(avionics_sim::Bilinear_interp::get1DLUTelementsFromString(yValsString, &outVect));

        // Check returned values
        EXPECT_EQ(outVect[0], 0.0f); 
        EXPECT_EQ(outVect[1], 20.0f); 
        EXPECT_EQ(outVect[2], 30.0f); 
        EXPECT_EQ(outVect[3], 40.0f); 
        EXPECT_EQ(outVect[4], 50.0f); 
        EXPECT_EQ(outVect[5], 60.0f); 
        EXPECT_EQ(outVect[6], 70.0f); 
    }

    TEST_F(BilinearInterp_UnitTest, get2DLUTelementsFromString)
    {
        std::string badVals1 = "0.0,596.92,1193.6,1790.4,,2983.8,3580.5,4177.2,4774.0,5370.7,5967.4,6564.1,7160.8,7757.6,8354.3,8951.0,8951.0;0.0,606.61,1212.8,1819.0,2425.2,3031.4,3637.6,4243.8,4850.1,5456.3,6062.5,6668.7,7274.9,7881.1,8487.3,9093.5,9093.5;0.0,2265.4,2719.2,3173.0,3626.7,4080.5,4534.3,4988.1,5441.9,5895.6,6349.4,6803.2,7257.0,7710.8,8164.5,8618.3,9072.1;0.0,3115.6,3511.0,3906.4,4301.8,4697.1,5092.5,5487.9,5883.3,6278.7,6674.1,7069.5,7464.9,7860.2,8255.6,8651.0,9046.4;0.0,3915.8,4256.5,4597.1,4937.8,5278.4,5619.1,5959.8,6300.4,6641.1,6981.7,7322.4,7663.1,8003.7,8344.4,8685.0,9025.7;0.0,4854.5,5134.3,5414.1,5693.9,5973.6,6253.4,6533.2,6813.0,7092.8,7372.6,7652.4,7932.2,8211.9,8491.7,8771.5,9051.3;0.0,5685.4,5915.7,6146.1,6376.4,6606.8,6837.1,7067.5,7297.8,7528.2,7758.5,7988.9,8219.2,8449.6,8679.9,8910.3,9140.6;";
        std::string badVals2 = "";
        std::string badVals3 = "0.0,596.92,1193.6,1790.4,2387.1,2983.8,3580.5,4177.2,4774.0,5370.7,5967.4,6564.1,7160.8,7757.6,8354.3,8951.0,8951.0;0.0,606.61,1212.8,1819.0,2425.2,3031.4,3637.6,4243.8,4850.1,5456.3,6062.5,6668.7,7274.9,7881.1,8487.3,9093.5,9093.5;0.0,2265.4,2719.2,3173.0,3626.7,4080.5,4534.3,4988.1,5441.9,5895.6,6349.4,6803.2,7257.0,7710.8,8164.5,8618.3,9072.1;0.0,3115.6,3511.0,3906.4,4301.8,4697.1,5092.5,5487.9,5883.3,6278.7,6674.1,7069.5,7464.9,7860.2,8255.6,8651.0,9046.4;0.0,3915.8,4256.5,4597.1,4937.8,5278.4,5619.1,5959.8,6300.4,6641.1,6981.7,7322.4,7663.1,8003.7,8344.4,8685.0,9025.7;0.0,4854.5,5134.3,5414.1,5693.9,5973.6,6253.4,6533.2,6813.0,7092.8,7372.6,7652.4,7932.2,8211.9,8491.7,8771.5,9051.3;0.0,5685.4,5915.7,6146.1,6376.4,6606.8,6837.1,7067.5,7297.8,7528.2,7758.5,7988.9,8219.2,8449.6,8679.9,8910.3,abc;";

        std::vector<std::vector<float>> outVect; 

        // Expect success == false for string with null element. 
        EXPECT_FALSE(avionics_sim::Bilinear_interp::get2DLUTelementsFromString(badVals1, &outVect));

        // Expect success == false for null string
        EXPECT_FALSE(avionics_sim::Bilinear_interp::get2DLUTelementsFromString(badVals2, &outVect));

        // Expect success == false for bad last element
        EXPECT_FALSE(avionics_sim::Bilinear_interp::get2DLUTelementsFromString(badVals3, &outVect));

        // Expect successful parse with proper values. 
        EXPECT_TRUE(avionics_sim::Bilinear_interp::get2DLUTelementsFromString(xValsString, &outVect)); 

        // Assert if returned vector is wrong size
        ASSERT_EQ(outVect.size(), 7);
        ASSERT_EQ(outVect[0].size(), 17); 

        // Check returned values
        // 0 kts
        EXPECT_FLOAT_EQ(outVect[0][0], 0.0);
        EXPECT_FLOAT_EQ(outVect[0][1], 596.92);
        EXPECT_FLOAT_EQ(outVect[0][2], 1193.6);
        EXPECT_FLOAT_EQ(outVect[0][3], 1790.4);
        EXPECT_FLOAT_EQ(outVect[0][4], 2387.1);
        EXPECT_FLOAT_EQ(outVect[0][5], 2983.8);
        EXPECT_FLOAT_EQ(outVect[0][6], 3580.5);
        EXPECT_FLOAT_EQ(outVect[0][7], 4177.2);
        EXPECT_FLOAT_EQ(outVect[0][8], 4774.0);
        EXPECT_FLOAT_EQ(outVect[0][9], 5370.7);
        EXPECT_FLOAT_EQ(outVect[0][10],5967.4);
        EXPECT_FLOAT_EQ(outVect[0][11],6564.1);
        EXPECT_FLOAT_EQ(outVect[0][12],7160.8);
        EXPECT_FLOAT_EQ(outVect[0][13],7757.6);
        EXPECT_FLOAT_EQ(outVect[0][14],8354.3);
        EXPECT_FLOAT_EQ(outVect[0][15],8951.0);
        EXPECT_FLOAT_EQ(outVect[0][16],8951.0);

        // 20 kts
        EXPECT_FLOAT_EQ(outVect[1][0], 0.0);
        EXPECT_FLOAT_EQ(outVect[1][1], 606.61);
        EXPECT_FLOAT_EQ(outVect[1][2], 1212.8);
        EXPECT_FLOAT_EQ(outVect[1][3], 1819.0);
        EXPECT_FLOAT_EQ(outVect[1][4], 2425.2);
        EXPECT_FLOAT_EQ(outVect[1][5], 3031.4);
        EXPECT_FLOAT_EQ(outVect[1][6], 3637.6);
        EXPECT_FLOAT_EQ(outVect[1][7], 4243.8);
        EXPECT_FLOAT_EQ(outVect[1][8], 4850.1);
        EXPECT_FLOAT_EQ(outVect[1][9], 5456.3);
        EXPECT_FLOAT_EQ(outVect[1][10],6062.5);
        EXPECT_FLOAT_EQ(outVect[1][11],6668.7);
        EXPECT_FLOAT_EQ(outVect[1][12],7274.9);
        EXPECT_FLOAT_EQ(outVect[1][13],7881.1);
        EXPECT_FLOAT_EQ(outVect[1][14],8487.3);
        EXPECT_FLOAT_EQ(outVect[1][15],9093.5);
        EXPECT_FLOAT_EQ(outVect[1][16],9093.5);

        // 30 kts
        EXPECT_FLOAT_EQ(outVect[2][0], 0.0);
        EXPECT_FLOAT_EQ(outVect[2][1], 2265.4);
        EXPECT_FLOAT_EQ(outVect[2][2], 2719.2);
        EXPECT_FLOAT_EQ(outVect[2][3], 3173.0);
        EXPECT_FLOAT_EQ(outVect[2][4], 3626.7);
        EXPECT_FLOAT_EQ(outVect[2][5], 4080.5);
        EXPECT_FLOAT_EQ(outVect[2][6], 4534.3);
        EXPECT_FLOAT_EQ(outVect[2][7], 4988.1);
        EXPECT_FLOAT_EQ(outVect[2][8], 5441.9);
        EXPECT_FLOAT_EQ(outVect[2][9], 5895.6);
        EXPECT_FLOAT_EQ(outVect[2][10],6349.4);
        EXPECT_FLOAT_EQ(outVect[2][11],6803.2);
        EXPECT_FLOAT_EQ(outVect[2][12],7257.0);
        EXPECT_FLOAT_EQ(outVect[2][13],7710.8);
        EXPECT_FLOAT_EQ(outVect[2][14],8164.5);
        EXPECT_FLOAT_EQ(outVect[2][15],8618.3);
        EXPECT_FLOAT_EQ(outVect[2][16],9072.1);

        // 40 kts
        EXPECT_FLOAT_EQ(outVect[3][0], 0.0);
        EXPECT_FLOAT_EQ(outVect[3][1], 3115.6);
        EXPECT_FLOAT_EQ(outVect[3][2], 3511.0);
        EXPECT_FLOAT_EQ(outVect[3][3], 3906.4);
        EXPECT_FLOAT_EQ(outVect[3][4], 4301.8);
        EXPECT_FLOAT_EQ(outVect[3][5], 4697.1);
        EXPECT_FLOAT_EQ(outVect[3][6], 5092.5);
        EXPECT_FLOAT_EQ(outVect[3][7], 5487.9);
        EXPECT_FLOAT_EQ(outVect[3][8], 5883.3);
        EXPECT_FLOAT_EQ(outVect[3][9], 6278.7);
        EXPECT_FLOAT_EQ(outVect[3][10],6674.1);
        EXPECT_FLOAT_EQ(outVect[3][11],7069.5);
        EXPECT_FLOAT_EQ(outVect[3][12],7464.9);
        EXPECT_FLOAT_EQ(outVect[3][13],7860.2);
        EXPECT_FLOAT_EQ(outVect[3][14],8255.6);
        EXPECT_FLOAT_EQ(outVect[3][15],8651.0);
        EXPECT_FLOAT_EQ(outVect[3][16],9046.4);

        // 50 kts
        EXPECT_FLOAT_EQ(outVect[4][0], 0.0);
        EXPECT_FLOAT_EQ(outVect[4][1], 3915.8);
        EXPECT_FLOAT_EQ(outVect[4][2], 4256.5);
        EXPECT_FLOAT_EQ(outVect[4][3], 4597.1);
        EXPECT_FLOAT_EQ(outVect[4][4], 4937.8);
        EXPECT_FLOAT_EQ(outVect[4][5], 5278.4);
        EXPECT_FLOAT_EQ(outVect[4][6], 5619.1);
        EXPECT_FLOAT_EQ(outVect[4][7], 5959.8);
        EXPECT_FLOAT_EQ(outVect[4][8], 6300.4);
        EXPECT_FLOAT_EQ(outVect[4][9], 6641.1);
        EXPECT_FLOAT_EQ(outVect[4][10],6981.7);
        EXPECT_FLOAT_EQ(outVect[4][11],7322.4);
        EXPECT_FLOAT_EQ(outVect[4][12],7663.1);
        EXPECT_FLOAT_EQ(outVect[4][13],8003.7);
        EXPECT_FLOAT_EQ(outVect[4][14],8344.4);
        EXPECT_FLOAT_EQ(outVect[4][15],8685.0);
        EXPECT_FLOAT_EQ(outVect[4][16],9025.7);

        // 60 kts
        EXPECT_FLOAT_EQ(outVect[5][0], 0.0);
        EXPECT_FLOAT_EQ(outVect[5][1], 4854.5);
        EXPECT_FLOAT_EQ(outVect[5][2], 5134.3);
        EXPECT_FLOAT_EQ(outVect[5][3], 5414.1);
        EXPECT_FLOAT_EQ(outVect[5][4], 5693.9);
        EXPECT_FLOAT_EQ(outVect[5][5], 5973.6);
        EXPECT_FLOAT_EQ(outVect[5][6], 6253.4);
        EXPECT_FLOAT_EQ(outVect[5][7], 6533.2);
        EXPECT_FLOAT_EQ(outVect[5][8], 6813.0);
        EXPECT_FLOAT_EQ(outVect[5][9], 7092.8);
        EXPECT_FLOAT_EQ(outVect[5][10],7372.6);
        EXPECT_FLOAT_EQ(outVect[5][11],7652.4);
        EXPECT_FLOAT_EQ(outVect[5][12],7932.2);
        EXPECT_FLOAT_EQ(outVect[5][13],8211.9);
        EXPECT_FLOAT_EQ(outVect[5][14],8491.7);
        EXPECT_FLOAT_EQ(outVect[5][15],8771.5);
        EXPECT_FLOAT_EQ(outVect[5][16],9051.3);

        // 70 kts
        EXPECT_FLOAT_EQ(outVect[6][0], 0.0);
        EXPECT_FLOAT_EQ(outVect[6][1], 5685.4);
        EXPECT_FLOAT_EQ(outVect[6][2], 5915.7);
        EXPECT_FLOAT_EQ(outVect[6][3], 6146.1);
        EXPECT_FLOAT_EQ(outVect[6][4], 6376.4);
        EXPECT_FLOAT_EQ(outVect[6][5], 6606.8);
        EXPECT_FLOAT_EQ(outVect[6][6], 6837.1);
        EXPECT_FLOAT_EQ(outVect[6][7], 7067.5);
        EXPECT_FLOAT_EQ(outVect[6][8], 7297.8);
        EXPECT_FLOAT_EQ(outVect[6][9], 7528.2);
        EXPECT_FLOAT_EQ(outVect[6][10],7758.5);
        EXPECT_FLOAT_EQ(outVect[6][11],7988.9);
        EXPECT_FLOAT_EQ(outVect[6][12],8219.2);
        EXPECT_FLOAT_EQ(outVect[6][13],8449.6);
        EXPECT_FLOAT_EQ(outVect[6][14],8679.9);
        EXPECT_FLOAT_EQ(outVect[6][15],8910.3);
        EXPECT_FLOAT_EQ(outVect[6][16],9140.6);
    }

    TEST_F(BilinearInterp_UnitTest, interpolate2D_withData)
    {
        std::vector<std::vector<float>> xVals, zVals; 
        std::vector<float> yVals; 

        float z; // Holds calculated value. 
        float expect; // Expected value

        // Load values into vectors. 
        ASSERT_TRUE(avionics_sim::Bilinear_interp::get2DLUTelementsFromString(xValsString, &xVals)); 
        ASSERT_TRUE(avionics_sim::Bilinear_interp::get1DLUTelementsFromString(yValsString, &yVals)); 
        ASSERT_TRUE(avionics_sim::Bilinear_interp::get2DLUTelementsFromString(zValsString, &zVals)); 

        // Instantiate a class member.
        avionics_sim::Bilinear_interp interp(xVals, yVals, zVals);
        
        // All tests are measured to within 1e-6. 

        // Test case 1
        z = 0.0f; expect = 0.168264052608138;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, 2900, 25, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 2
        z = 0.0f; expect = 0.024456462046209;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, 3000, 32.5, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 3
        z = 0.0f; expect = 2.04379665677043;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, 7000, 51.8, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 4
        z = 0.0f; expect = -0.0065027403;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, 0, 2.36894, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 5
        // Should return out of range error. 
        z = 0.0f; expect = 4.81338087423456;
        ASSERT_EQ((int)interp.interpolate2D(xVals, yVals, zVals, 8973.26953, 0.02074, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 6
        // Should return out of range error. 
        z = 0.0f; expect = 4.813;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, 8995.44043, -66.014, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 7
        z = 0.0f; expect = 0.065833032446202;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, 898.68610, 0.0, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 8
        z = 0.0f; expect = 0.0;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, 0.000, 0.000, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);
    
       // Test case 9
        z = 0.0f; expect = 0.0;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, -10.0, 0.000, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 10
        z = 0.0f; expect = -0.435;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, 0, 75.0, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 11
        z = 0.0f; expect = 4.83881;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, 10000, 1.0, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 11
        z = 0.0f; expect = 3.3587;
        ASSERT_EQ(interp.interpolate2D(xVals, yVals, zVals, 10000, 75.0, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

    }

    TEST_F(BilinearInterp_UnitTest, setXVals_String)
    {
        std::string badVals1 = "0.0,596.92,1193.6,1790.4,,2983.8,3580.5,4177.2,4774.0,5370.7,5967.4,6564.1,7160.8,7757.6,8354.3,8951.0,8951.0;0.0,606.61,1212.8,1819.0,2425.2,3031.4,3637.6,4243.8,4850.1,5456.3,6062.5,6668.7,7274.9,7881.1,8487.3,9093.5,9093.5;0.0,2265.4,2719.2,3173.0,3626.7,4080.5,4534.3,4988.1,5441.9,5895.6,6349.4,6803.2,7257.0,7710.8,8164.5,8618.3,9072.1;0.0,3115.6,3511.0,3906.4,4301.8,4697.1,5092.5,5487.9,5883.3,6278.7,6674.1,7069.5,7464.9,7860.2,8255.6,8651.0,9046.4;0.0,3915.8,4256.5,4597.1,4937.8,5278.4,5619.1,5959.8,6300.4,6641.1,6981.7,7322.4,7663.1,8003.7,8344.4,8685.0,9025.7;0.0,4854.5,5134.3,5414.1,5693.9,5973.6,6253.4,6533.2,6813.0,7092.8,7372.6,7652.4,7932.2,8211.9,8491.7,8771.5,9051.3;0.0,5685.4,5915.7,6146.1,6376.4,6606.8,6837.1,7067.5,7297.8,7528.2,7758.5,7988.9,8219.2,8449.6,8679.9,8910.3,9140.6;";
        std::string badVals2 = "";

        float z; // Holds calculated value. 
        float expect; // Expected value

        std::vector<std::vector<float>> xVals; 

        avionics_sim::Bilinear_interp interp; 

        // Should return BilinearInterp::INTERP_ERROR_NO_LUT because no values. 
        ASSERT_EQ(interp.interpolate2D(0.0, 0.0, &z), avionics_sim::Bilinear_interp::INTERP_ERROR_NO_LUT);

        // Should return false because no data yet. 
        ASSERT_FALSE(interp.getX(&xVals));

        ASSERT_FALSE(interp.setXVals(badVals1)); // Set xVals via bad values. 
        ASSERT_FALSE(interp.setXVals(badVals2)); // Set xVals via null values. 

        ASSERT_TRUE(interp.setXVals(xValsString)); // Set xVals via string. 

        ASSERT_TRUE(interp.getX(&xVals)); // Should have values now. 

        // Assert if returned vector is wrong size
        ASSERT_EQ(xVals.size(), 7);
        ASSERT_EQ(xVals[0].size(), 17); 

        // Should return BilinearInterp::INTERP_ERROR_NO_LUT because no values. 
        ASSERT_EQ(interp.interpolate2D(0.0, 0.0, &z), avionics_sim::Bilinear_interp::INTERP_ERROR_NO_LUT);

        // Check returned values
        // 0 kts
        EXPECT_FLOAT_EQ(xVals[0][0], 0.0);
        EXPECT_FLOAT_EQ(xVals[0][1], 596.92);
        // 60 kts
        EXPECT_FLOAT_EQ(xVals[5][0], 0.0);
        EXPECT_FLOAT_EQ(xVals[5][1], 4854.5);
        // 70 kts
        EXPECT_FLOAT_EQ(xVals[6][0], 0.0);
        EXPECT_FLOAT_EQ(xVals[6][1], 5685.4);
    }

    TEST_F(BilinearInterp_UnitTest, setYVals_String)
    {
        std::string badVals1 = "0,20.0,30.0,,50.0,60.0,70.0";
        std::string badVals2 = "";

        float z; // Holds calculated value. 
        float expect; // Expected value

        std::vector<float> yVals; 

        avionics_sim::Bilinear_interp interp; 

        // Should return BilinearInterp::INTERP_ERROR_NO_LUT because no values. 
        ASSERT_EQ(interp.interpolate2D(0.0, 0.0, &z), avionics_sim::Bilinear_interp::INTERP_ERROR_NO_LUT);

        // Should return false because no data yet. 
        ASSERT_FALSE(interp.getY(&yVals));

        ASSERT_FALSE(interp.setYVals(badVals1)); // Set xVals via bad values. 
        ASSERT_FALSE(interp.setYVals(badVals2)); // Set xVals via null values. 

        ASSERT_TRUE(interp.setYVals(yValsString)); // Set xVals via string. 

        ASSERT_TRUE(interp.getY(&yVals)); // Should have values now. 

        // Assert if returned vector is wrong size
        ASSERT_EQ(yVals.size(), 7);

        // Should return BilinearInterp::INTERP_ERROR_NO_LUT because no values. 
        ASSERT_EQ(interp.interpolate2D(0.0, 0.0, &z), avionics_sim::Bilinear_interp::INTERP_ERROR_NO_LUT);

        // Check returned values
        // 0 kts
        EXPECT_FLOAT_EQ(yVals[0], 0.0);
        EXPECT_FLOAT_EQ(yVals[1], 20.0);
        EXPECT_FLOAT_EQ(yVals[6], 70.0); 
    }

    TEST_F(BilinearInterp_UnitTest, setZVals_String)
    {
        std::string badVals1 = "0.0,596.92,1193.6,1790.4,,2983.8,3580.5,4177.2,4774.0,5370.7,5967.4,6564.1,7160.8,7757.6,8354.3,8951.0,8951.0;0.0,606.61,1212.8,1819.0,2425.2,3031.4,3637.6,4243.8,4850.1,5456.3,6062.5,6668.7,7274.9,7881.1,8487.3,9093.5,9093.5;0.0,2265.4,2719.2,3173.0,3626.7,4080.5,4534.3,4988.1,5441.9,5895.6,6349.4,6803.2,7257.0,7710.8,8164.5,8618.3,9072.1;0.0,3115.6,3511.0,3906.4,4301.8,4697.1,5092.5,5487.9,5883.3,6278.7,6674.1,7069.5,7464.9,7860.2,8255.6,8651.0,9046.4;0.0,3915.8,4256.5,4597.1,4937.8,5278.4,5619.1,5959.8,6300.4,6641.1,6981.7,7322.4,7663.1,8003.7,8344.4,8685.0,9025.7;0.0,4854.5,5134.3,5414.1,5693.9,5973.6,6253.4,6533.2,6813.0,7092.8,7372.6,7652.4,7932.2,8211.9,8491.7,8771.5,9051.3;0.0,5685.4,5915.7,6146.1,6376.4,6606.8,6837.1,7067.5,7297.8,7528.2,7758.5,7988.9,8219.2,8449.6,8679.9,8910.3,9140.6;";
        std::string badVals2 = "";

        float z; // Holds calculated value. 
        float expect; // Expected value

        std::vector<std::vector<float>> zVals; 

        avionics_sim::Bilinear_interp interp; 

        // Should return BilinearInterp::INTERP_ERROR_NO_LUT because no values. 
        ASSERT_EQ(interp.interpolate2D(0.0, 0.0, &z), avionics_sim::Bilinear_interp::INTERP_ERROR_NO_LUT);

        // Should return false because no data yet. 
        ASSERT_FALSE(interp.getZ(&zVals));

        ASSERT_FALSE(interp.setZVals(badVals1)); // Set xVals via bad values. 
        ASSERT_FALSE(interp.setZVals(badVals2)); // Set xVals via null values. 

        ASSERT_TRUE(interp.setZVals(zValsString)); // Set xVals via string. 

        ASSERT_TRUE(interp.getZ(&zVals)); // Should have values now. 

        // Assert if returned vector is wrong size
        ASSERT_EQ(zVals.size(), 7);
        ASSERT_EQ(zVals[0].size(), 17); 

        // Should return BilinearInterp::INTERP_ERROR_NO_LUT because no values. 
        ASSERT_EQ(interp.interpolate2D(0.0, 0.0, &z), avionics_sim::Bilinear_interp::INTERP_ERROR_NO_LUT);

        // Check returned values
        // 0 kts
        EXPECT_FLOAT_EQ(zVals[0][0], 0.0);
        EXPECT_FLOAT_EQ(zVals[0][1], 0.0365);
        // 60 kts
        EXPECT_FLOAT_EQ(zVals[5][0], -0.3076);
        EXPECT_FLOAT_EQ(zVals[5][1], -0.307);
        // 70 kts
        EXPECT_FLOAT_EQ(zVals[6][0], -0.435);
        EXPECT_FLOAT_EQ(zVals[6][1], -0.435);
    }

    TEST_F(BilinearInterp_UnitTest, interpolate2D)
    {
        std::vector<std::vector<float>> xVals, zVals; 
        std::vector<float> yVals; 

        float z; // Holds calculated value. 
        float expect; // Expected value

        // Load values into vectors. 
        ASSERT_TRUE(avionics_sim::Bilinear_interp::get2DLUTelementsFromString(xValsString, &xVals)); 
        ASSERT_TRUE(avionics_sim::Bilinear_interp::get1DLUTelementsFromString(yValsString, &yVals)); 
        ASSERT_TRUE(avionics_sim::Bilinear_interp::get2DLUTelementsFromString(zValsString, &zVals)); 

        avionics_sim::Bilinear_interp interp;

        // Should return BilinearInterp::INTERP_ERROR_NO_LUT because no values. 
        ASSERT_EQ(interp.interpolate2D(0.0, 0.0, &z), avionics_sim::Bilinear_interp::INTERP_ERROR_NO_LUT);  

        // Still dont have all values. 
        interp.setXVals(xVals); 
        ASSERT_EQ(interp.interpolate2D(0.0, 0.0, &z), avionics_sim::Bilinear_interp::INTERP_ERROR_NO_LUT);

        // Still dont have all values.
        interp.setYVals(yVals); 
        ASSERT_EQ(interp.interpolate2D(0.0, 0.0, &z), avionics_sim::Bilinear_interp::INTERP_ERROR_NO_LUT);

        // Should now have all needed values. 
        interp.setZVals(zVals); 
        ASSERT_EQ(interp.interpolate2D(0.0, 0.0, &z), avionics_sim::Bilinear_interp::INTERP_SUCCESS);

        // Test case 1
        // Just to make sure it works after loading variables at runtime. 
        z = 0.0f; expect = 0.168264052608138;
        ASSERT_EQ(interp.interpolate2D(2900, 25, &z), avionics_sim::Bilinear_interp::INTERP_SUCCESS);
        EXPECT_NEAR(z, expect, 1e-6);

        interp = avionics_sim::Bilinear_interp(xVals, yVals, zVals);

        // Assert if unsuccessful, cannot continue. 
        ASSERT_EQ(interp.interpolate2D(2900, 25, &z), avionics_sim::Bilinear_interp::INTERP_SUCCESS);

        // All tests are measured to within 1e-6. 

        // Test case 1
        z = 0.0f; expect = 0.168264052608138;
        ASSERT_EQ(interp.interpolate2D(2900, 25, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 2
        z = 0.0f; expect = 0.024456462046209;
        ASSERT_EQ(interp.interpolate2D(3000, 32.5, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 3
        z = 0.0f; expect = 2.04379665677043;
        ASSERT_EQ(interp.interpolate2D(7000, 51.8, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 4
        z = 0.0f; expect = -0.0065027403;
        ASSERT_EQ(interp.interpolate2D(0, 2.36894, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 5
        // Should return out of range error. 
        z = 0.0f; expect = 4.81338087423456;
        ASSERT_EQ((int)interp.interpolate2D(8973.26953, 0.02074, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 6
        // Should return out of range error. 
        z = 0.0f; expect = 4.813;
        ASSERT_EQ(interp.interpolate2D(8995.44043, -66.014, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 7
        z = 0.0f; expect = 0.065833032446202;
        ASSERT_EQ(interp.interpolate2D(898.68610, 0.0, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 8
        z = 0.0f; expect = 0.0;
        ASSERT_EQ(interp.interpolate2D(0.000, 0.000, &z), 0);
        EXPECT_NEAR(z, expect, 1e-6);
    
        // Test case 9
        z = 0.0f; expect = 0.0;
        ASSERT_EQ(interp.interpolate2D(-10.0, 0.000, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 10
        z = 0.0f; expect = -0.435;
        ASSERT_EQ(interp.interpolate2D(0, 75.0, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 11
        z = 0.0f; expect = 4.83881;
        ASSERT_EQ(interp.interpolate2D(10000, 1.0, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);

        // Test case 12
        z = 0.0f; expect = 3.3587;
        ASSERT_EQ(interp.interpolate2D(10000, 75.0, &z), -1);
        EXPECT_NEAR(z, expect, 1e-6);
    }
}