TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#
#  #----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#  #DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install)
# # databases, templates, substitutions like this
# #
#
# llrfHlsAsyn
DB += fastPV.db
DB += llrfHlsAsyn_NC_XTCAV.db
DB += llrfHlsAsyn_SC_XTCAV.db
DB += llrfHlsAsyn_SC_STCAV.db
DB += ued_llrfHlsAsyn.db


DB_INSTALLS += $(BSADRIVER)/db/bsa.db


# =======================================================================

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_TEMPLATE = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
