simple_test

Syntax: result = simple_test(th1,th2,th3,th4,th5,th6,team_num)
Manually pick joint angle to test whether the code could run or not.
Input: th1,th2,th3,th4,th5,th6 are joint angles
       team_num is the team's number in integer
Output: The possible results include 'NaN', 'Wrong' and 'Right'.



puma_ik_test

Syntax: [score1 score2] = puma_ik_test(team_num)
Automatically generate 100 groups of joint angles to test ik.
Input: team_num
Output: score1: without current configuration
        score2: with current configuration
        both of them are integers between 0 and 100
This function will also print out the number of NaN and wrong solutions 
for both scores (with and without current configuration).



IMPORTANT: This tester function assumes the point on the robot that you want to place at x y z is that of the origin of frame 6, not the LED.