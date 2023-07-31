cd %~dp0
docker build . -t nurbs_intersection_on_ubuntu
docker run -it nurbs_intersection_on_ubuntu
