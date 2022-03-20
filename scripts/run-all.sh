for f in $1/*.ini
do
	echo "Processing $f"
    build/engine $f
done