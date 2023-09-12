
viamboatbase: *.go cmd/module/*.go
	go build -o viamboatbase cmd/module/cmd.go

test:
	go test

lint:
	gofmt -w -s .

module: viamboatbase
	tar czf module.tgz viamboatbase
