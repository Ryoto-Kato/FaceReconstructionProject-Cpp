mkdir out
EXPRESSION="Smile"
# Loop over persons in EURECOM without glasses
for i in 21 42 44 50 51
do
    mkdir ./out/s1_p$i\_$EXPRESSION
    ../build/FaceReconstruction -p $i --expression $EXPRESSION > ./out/s1_p$i\_$EXPRESSION/command_line_log.txt
    cp -r ../output/ out/s1_p$i\_$EXPRESSION/output
    mv detected_landmark.png out/s1_p$i\_$EXPRESSION/detected_landmark.png
    mv PS_considered_landmarks.png out/s1_p$i\_$EXPRESSION/PS_considered_landmarks.png
    python visualisation.py $i "$EXPRESSION" True
    # for hybrid graphics: prime-run python visualisation.py $i "$EXPRESSION" True
done