# ELEC390
Queen's University 3rd year design project course on a small autonomous vehicle. 

Website for contest information: https://queensuca.sharepoint.com/teams/CCT-938446/SitePages/LearningTeamHome.aspx

# Tasks
- [ ] prioritize tasks
- [ ] plan times to do tasks
- [ ] finish setting up PiCarX as described on the SharePoint
- [ ] set up programming/testing environment on the PiCarX so that we can rapidly test all features
    - [ ] figure out software development environment (combining Python and C)
    - [ ] make PiCarX remote controllable or at least programmable (figure out the drive control interface)
    - [ ] make PiCarX able to send API calls to VPFS and store results
    - [ ] make PiCarX able to sense data from all sensors simultaneously
    - [ ] make PiCarX able to record sensor data (ideally so that we can live stream sensor data to a laptop, perhaps at least the head camera feed)
    - [ ] make PiCarX able to run software on both Raspberry Pi and Coral TPU
    - [ ] set up high-level software interface between sensors and drive controls and API calls so that we can incrementally add in each new feature as it is developed
- [ ] Test physical capabilities of drive controls
    - [ ] turn radius at different speeds and road surfaces
    - [ ] top speed on different road surfaces
    - [ ] minimum braking distance at different speeds and road surfaces from the point in time when braking is initiated in software
        - [ ] figure out how we are going to measure this (perhaps using the grayscale line detector?)
- [ ] determine detection strategy for different objects/patterns
    - [ ] road signs (stop, no entry, yield, one way)
    - [ ] road markings (road boundary, road center line, intersection stop line, cross walk, one way arrow)
    - [ ] road obstacles (vehicles, duck pedestrians, other objects)
- [ ] represent the map of Quackston in software
- [ ] put break and turn signal lights on PiCarX
- [ ] make break light system work
    - [ ] specify what states of the vehicle will trigger brake lights
- [ ] make turn signal light system work
    - [ ] specify blinking rate and high-level software control interface
- [ ] make the duck lift controllable in software
- [ ] break down more tasks here
    - [ ] route planning (optimal sequence of road and intersections between two destinations on the map)
    - [ ] path planning (specific path on the road to be followed by drive controls)
    - [ ] ride selection
    - [ ] position generation
    - [ ] represent the reward system / road rules in software
    - [ ] be able to infer vehicle position based on sensor data
    - [ ] be able to infer other object positions base on sensor data
    - [ ] implement autonomous driving capabilities

# Complete
- [x] Project Proposal Report 📅 2025-01-26 ✅ 2025-01-26
    - [x] Write Sections📅 2025-01-25 
        - [x] Executive Summary @Hendrix ✅ 2025-01-26
        - [x] Project Management @Hendrix ✅ 2025-01-26
        - [x] Finish up data sources (where to get data) @Hendrix ✅ 2025-01-26
        - [x] Conclusions @Matt
    - [x] Review and edit sections 🛫 2025-01-25 📅 2025-01-26 ✅ 2025-01-26
        - [x] Jacob ✅ 2025-01-26
        - [x] Hendrix ✅ 2025-01-26
        - [x] Luke ✅ 2025-01-26
        - [x] Matt ✅ 2025-01-26
    - [x] ensure consistent formatting
        - [x] remove our names in headings 
        - [x] paragraph spacing and indenting (vspace vs. newline vs. `\\` vs. `\\\\`?) (decided on indenting all but the first paragraph in a section)
        - [x] technologies and tools table formatting
            - [x] Hendrix needs to update python libraries
        - [x] references 
            - [x] Jacob
            - [x] Hendrix
            - [x] Luke
            - [x] Matt
    - [x] submit [on OnQ](https://onq.queensu.ca/d2l/lms/dropbox/user/folder_submit_files.d2l?db=409840&grpid=979636&isprv=0&bp=0&ou=938446) ✅ 2025-01-26
- [x] Design Headlight Board for taxi
- [x] Create Lift for duck passengers
