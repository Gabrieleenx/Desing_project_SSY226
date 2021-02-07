all: cluonDataStructures_pb2.py porpoiseMessageSet_pb2.py

cluonDataStructures.proto: cluonDataStructures.odvd
	cluon-msc --proto --out=cluonDataStructures.proto cluonDataStructures.odvd

cluonDataStructures_pb2.py: cluonDataStructures.proto
	protoc --python_out=. cluonDataStructures.proto

porpoiseMessageSet.proto: porpoiseMessageSet.odvd
	cluon-msc --proto --out=porpoiseMessageSet.proto porpoiseMessageSet.odvd

porpoiseMessageSet_pb2.py: porpoiseMessageSet.proto
	protoc --python_out=. porpoiseMessageSet.proto

clean:
	rm -f *_pb2.py *pyc *.proto 
