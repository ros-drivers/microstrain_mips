.PHONY: all build build-shell clean

# These variables should be overridden in CI builds to configure this build
version ?= latest
docker ?= docker

# Just set some directory values to support out of tree builds
makefile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
docker_dir := $(dir $(makefile_path))
project_dir := $(abspath $(docker_dir)/..)
project_name := $(shell echo $(notdir $(project_dir)) | tr A-Z a-z)
build_dir := $(docker_dir)/build
devel_dir := $(docker_dir)/devel
docker_catkin_root := /home/$(project_name)
docker_catkin_src_dir := $(docker_catkin_root)/src
docker_catkin_build_dir := $(docker_catkin_root)/build
docker_catkin_devel_dir := $(docker_catkin_root)/devel

# Set up some variables for the docker build
dockerfile := $(docker_dir)/Dockerfile
build_args := --build-arg http_proxy --build-arg https_proxy --build-arg no_proxy
run_args := -e http_proxy -e https_proxy -e no_proxy
run_mounts := -v "$(project_dir)/ros_mscl:$(docker_catkin_src_dir)/ros_mscl" -v "$(project_dir)/mscl_msgs:$(docker_catkin_src_dir)/mscl_msgs" -v "$(build_dir):$(docker_catkin_build_dir)" -v "$(devel_dir):$(docker_catkin_devel_dir)"
image_name := $(project_name):$(version)
image_artifact := .image-$(project_name)

all: build
build: $(image_artifact)
$(image_artifact): $(dockerfile)
	@$(docker) build \
		-t $(image_name) \
		-f $(dockerfile) \
		$(build_args) \
		$(project_dir)
	@echo $(image_name) > $@

build-shell: $(image_artifact)
	@$(docker) run \
		-it \
		--rm \
		-w $(docker_catkin_root) \
		$(run_args) \
		$(run_mounts) \
		$$(cat $<)

clean: $(image_artifact)
	@$(docker) run \
		-it \
		--rm \
		-w $(docker_catkin_root) \
    --entrypoint="/bin/bash" \
		$(run_args) \
		$(run_mounts) \
		$$(cat $<) -c " \
			rm -rf '$(docker_catkin_build_dir)/*' '$(docker_catkin_devel_dir)/*' \
		"
	@rm -rf "$(build_dir)" "$(devel_dir)"
	@rm -f "$(image_artifact)"
	@docker ps -a | grep "$(project_name)" | grep $(version) | tr -s " " | cut -d' ' -f1 | xargs docker rm -f || echo "No containers to remove"
	@docker images | grep "$(project_name)" | grep $(version) | tr -s " " | cut -d' ' -f1 | xargs docker rmi -f || echo "No images to remove"
