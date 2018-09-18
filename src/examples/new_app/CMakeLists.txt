px4_add_module(
    MODULE examples__new_app
    MAIN new_app
    STACK_MAIN 2000
    SRCS
        new_app.c
    DEPENDS
        platforms__common
    )
