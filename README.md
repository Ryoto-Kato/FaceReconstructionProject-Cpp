# Organization
- .gitignore
  - If you want to ignore your own personal folder or files within this root directory, you can ignore them by rename it with "**personal**"

# Team
## Members [[team_list](https://docs.google.com/spreadsheets/d/1KcEIfJHHQVqG12gHMNQo_0s5yTAw82hNx3K4T5NtPMY/edit#gid=0)]

    - Ben Robert Sturgis	03776460	
    - Kevin	Qu	          03730587	
    - Marco	Busch	        03779574	
    - Ryoto	Kato	        03767467

# Abstract
Faithful 3D face reconstruction and the synthesis of reliable facial expressions are an essential foundation for the generation of photorealistic virtual avatars. The entertainment industry has addressed this task by having experienced artists manually create these avatars. While they are able to deliver impressive results, it is rather impractical because of the time- and labor-intensive nature of their work. In our project, we present an approach for 3D face reconstruction which does not require labor-intensive manual work. Based on a parametric face model, we perform an analysis-by-synthesis loop to reconstruct the 3D face from a single RGB-D input image. Furthermore, we apply an expression transfer between a source person and a target person.

# Tasks
- Week 1 (10.06 - 16.06)
  - [ ] Decided on final dataset, got access and familiarized ourselves with it
  - [ ] Completed implementation of input/output functions as well as data transformation and prepro-
  cessing
  - [ ] weekly report
- Week 2 (17.06 - 23.06)
  - [ ] Finished creation of a code template for orientation and better distribution of tasks, which we keep running during the entire development process
  - [ ] weekly report
- Week 3 (24.06 - 30.06)
  - [ ] Integrated suitable off-the-shelf facial landmark detector
  - [ ] Implemented Procrustes algorithm to get an initial coarse alignment of the face model and the input RGB-D image using the obtained landmarks
  - [ ] weekly report
- Week 4 (01.07 - 07.07)
  - [ ] Defined the energy function using the Ceres library
  - [ ] weekly report
- Week 5 (08.07 - 14.07)
  - [ ] Implemented the transfer of facial expressions of a source to a target person
  - [ ] weekly report
- Week 6 (15.07 - 21.07)
  - [ ] Fixed remaining bugs and code cleanup
  - [ ] Performed experiments to evaluate the developed face reconstruction
  - [ ] Optional: Integrated rendering pipeline using OpenGL
  - [ ] weekly report
- Week 7 (22.07 - 28.07)
  - [ ] Completion of the final report and presentation video
  - [ ] weekly report
- Week 8 (29.07 - 31.07)
  - [ ] Prepared for presentation
  - [ ] weekly report

# Proposals (Deadline: June 9)
- Overleaf (CVPR)
- Time line
      1. Individually write respective section by Thursday 23:59
      2. Check them individually and note comments
      3. Meeting on Friday
      4. Finalize the proposal

# Weekly report (Every Friday)
- Format: Google Docs
- Content
- about 1 page
    - What you have accomplished in the
    week
    - Which problems did you encounter
    - If possible, show some intermediate
    results
    - Outline plan for the next week

# Final Report (Deadline: July 28)
- Content
  - 4 pages including figures and tables, excluding references
  - Should be structured like a paper
    - Introduction (Motivation)
    - Related Work (What has been done before)
    - Method (What we did)
    - Results (Quantitative & Qualitative)
    - Conclusion (What was achieved, what can be achieved in future)
- Deadline
  - Friday July 28 23:59
