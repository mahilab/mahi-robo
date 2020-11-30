macro(mahi_robo_example target)
    # create executable
    add_executable(${target} "ex_${target}.cpp")
    # set dependencies
    target_link_libraries(${target} mahi::robo)
    # add install rule
    install(TARGETS ${target}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    set_target_properties(${target} PROPERTIES FOLDER "Examples")
    set_target_properties(${target} PROPERTIES DEBUG_POSTFIX -d)
endmacro(mahi_robo_example)

mahi_robo_example(trajectory)