#!/bin/bash
var="$(curl -H "X-Auth-Token: 4MdU_nYBj1hrMdivEa4gYtHRpfobAXpRduQe8FkVwU2" -H "X-User-Id: MoharpFGNkjhmBFzi" http://localhost:3000/api/v1/videos)"
echo $var
IFS=';' read -ra ADDR <<< "$var"
for i in "${ADDR[@]}"; do
    xdg-open "$i" 
    sleep 10
done

