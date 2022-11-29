#!/bin/bash

submission_file="submission/HW4"
hw_dir="HW4"
mkdir -p "./${submission_file}"

find "./${hw_dir}" -name "ekf.py" -exec cp -rf "{}" "./${submission_file}" \;
find "./${hw_dir}" -name "particle_filter.py" -exec cp -rf "{}" "./${submission_file}" \;
find "./${hw_dir}" -name "turtlebot_model.py" -exec cp -rf "{}" "./${submission_file}" \;

cd "./submission"
zip -r hw4.zip HW4
cd ..
mv "./submission/hw4.zip" .
# zip -r hw4.zip "./${submission_file}"
