var="$(curl -H "X-Auth-Token: 4MdU_nYBj1hrMdivEa4gYtHRpfobAXpRduQe8FkVwU2" -H "X-User-Id: MoharpFGNkjhmBFzi" http://localhost:3000/api/v1/signals)"
echo $var
if [ "$var" == "pushchanges" ]; then
  (/home/capstone38/Desktop/electricTree/scripts/downloadAndContinue.sh)
fi
