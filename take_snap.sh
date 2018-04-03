#!/bin/sh
rosservice call --wait /saver/save &
exec "$@"
