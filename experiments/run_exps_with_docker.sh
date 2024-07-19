echo "Run the experiments."
docker build -t compute_exps --progress=plain .
docker run -m 14g -d --name compute_exps compute_exps

chmod -R 777 ./results/compute_results
echo "Finished. Copy the logs folders on the local machine."
docker cp compute_exps:/app/exp/compute_results/logs ./results/compute_results
chmod -R 755 ./results/compute_results
echo "Give the ownership of the folder to $1"
chown -R $1 ./results/compute_results/logs
chmod -R 755 ./results/compute_results/logs

docker stop compute_exps
docker rm compute_exps