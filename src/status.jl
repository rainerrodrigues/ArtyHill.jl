# Status codes returned by every backend / step function (Int32, 0 = OK).
# Plain integer codes keep the hot path free of exceptions, which allocate.

const HIL_OK           = Int32(0)
const HIL_ERR_IO       = Int32(-1)   # short read/write, bus error
const HIL_ERR_NOT_INIT = Int32(-2)   # backend used before init_backend!
const HIL_ERR_SYS      = Int32(-3)   # a POSIX / library call failed during setup
