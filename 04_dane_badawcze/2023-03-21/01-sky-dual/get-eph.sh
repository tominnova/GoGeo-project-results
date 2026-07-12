#day=160

day=$(date +%j)
year=$(date +%Y)
yy=$(date +%y)

exdate=2023-03-22
day=$(date -d ${exdate} +%j)
year=$(date -d ${exdate} +%Y)
yy=$(date -d ${exdate} +%y)

brdc_fname=brdc${day}0\.${yy}n
#brdc_fname_z=${brdc_fname}.Z
brdc_fname_z=${brdc_fname}.gz
rm -f ${brdc_fname_z}
# musi być plik ~/.netrc
wget --auth-no-challenge "https://cddis.nasa.gov/archive/gnss/data/daily/${year}/brdc/${brdc_fname_z}"
uncompress -f ${brdc_fname_z}

